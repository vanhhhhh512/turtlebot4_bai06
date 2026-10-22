"""Recognise a finger wave: extend, curl, extend twice before confirming."""
from collections import deque
from dataclasses import dataclass
from statistics import median
import math


@dataclass(frozen=True)
class FingerPose:
    extension: tuple
    thumb_open: bool = True

    @property
    def open(self):
        return self.thumb_open and sum(value >= 0.70 for value in self.extension) >= 3

    @property
    def curled(self):
        return sum(value <= 0.40 for value in self.extension) >= 3


def finger_pose(landmarks):
    """Measure fingers relative to wrist/PIP distances, independent of position.

    Three fingers must agree; moving a rigid palm or showing one/two extended
    fingers does not complete the required two curl-and-extend cycles.
    """
    if len(landmarks) != 21:
        return None

    def distance(a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2
                         + (getattr(a, 'z', 0) - getattr(b, 'z', 0)) ** 2)

    if distance(landmarks[5], landmarks[17]) < 0.025:
        return None
    values = []
    for pip, tip in ((6, 8), (10, 12), (14, 16), (18, 20)):
        base = distance(landmarks[0], landmarks[pip])
        if base < 1e-5:
            return None
        ratio = distance(landmarks[0], landmarks[tip]) / base
        values.append(max(0.0, min(1.0, (ratio - 0.65) / 0.85)))
    a, b, c = landmarks[2], landmarks[3], landmarks[4]
    u = (a.x-b.x, a.y-b.y, getattr(a, 'z', 0)-getattr(b, 'z', 0))
    v = (c.x-b.x, c.y-b.y, getattr(c, 'z', 0)-getattr(b, 'z', 0))
    norm = math.sqrt(sum(x*x for x in u)*sum(x*x for x in v))
    thumb_open = (norm > 1e-8 and sum(x*y for x,y in zip(u,v))/norm < -0.55
                  and distance(c, landmarks[17]) > 1.05*distance(b, landmarks[17]))
    return FingerPose(tuple(values), thumb_open)


class FingerWaveDetector:
    """Arm with five open fingers, then measure relative finger flexion.

    A three-sample median rejects single-frame spikes. The fingers need not
    close into a fist: three must bend by min_swing then reopen, twice.
    """
    def __init__(self, window_s=4.0, min_cycles=2, min_swing=0.22, gap_s=0.25):
        self.window_s = window_s
        self.min_cycles = min_cycles
        self.min_swing = min_swing
        self.gap_s = gap_s
        self.last_emit = -1e9
        self.reset()

    def reset(self):
        self.phase = ''
        self.started_at = None
        self.last_at = -1e9
        self.cycles = 0
        self.samples = deque(maxlen=3)
        self.baseline = None
        self.trough = None
        self.participants = ()
        self.arm_count = 0

    @property
    def collecting(self):
        return bool(self.phase)

    @property
    def hint(self):
        if not self.phase:
            return 'Open 5 fingers first'
        return 'Extend fingers again' if self.phase == 'CURL' else 'Wiggle fingers gently'

    def update(self, now, pose, open_hint=False):
        if (now - self.last_at > self.gap_s
                or self.started_at is not None and now - self.started_at > self.window_s):
            self.reset()
        self.last_at = now
        self.samples.append(pose.extension)
        smooth = tuple(median(v[i] for v in self.samples) for i in range(4))
        if self.baseline is None:
            ready = (pose.thumb_open or open_hint) and sum(v >= .55 for v in smooth) >= 3
            self.arm_count = self.arm_count+1 if ready else 0
            if self.arm_count >= 2:
                self.baseline = smooth
                self.phase = 'OPEN'
            return False
        if self.phase == 'OPEN':
            if self.cycles == 0:
                self.baseline = tuple(max(a,b) for a,b in zip(self.baseline,smooth))
            bent = tuple(i for i in range(4) if self.baseline[i]-smooth[i] >= self.min_swing)
            if len(bent) >= 3:
                if self.started_at is None:
                    self.started_at = now
                self.phase = 'CURL'
                self.trough = smooth
                self.participants = bent
        else:
            self.trough = tuple(min(a,b) for a,b in zip(self.trough,smooth))
            reopened = sum(smooth[i]-self.trough[i] >= .16
                           and smooth[i] >= self.baseline[i]-.18 for i in self.participants)
            if reopened >= 3:
                self.cycles += 1
                self.phase = 'OPEN'
                self.baseline = tuple(max(v,b-.12) for v,b in zip(smooth,self.baseline))
                if (self.cycles >= self.min_cycles and now-self.started_at >= .35
                        and now-self.last_emit > 1.0):
                    self.last_emit = now
                    self.reset()
                    return True
        return False


class GestureRouter:
    """Motion and static poses share landmarks, not each other's class labels."""
    def __init__(self, confirm_window_s=3.0, **wave_options):
        self.wave = FingerWaveDetector(**wave_options)
        self.confirm_window_s = min(max(float(confirm_window_s), .5), 3.0)
        self.transition_until = 0.0
        self.last_signal_at = -1e9
        self.static_label = None
        self.static_since = 0.0

    def reset(self):
        self.wave.reset()
        self.transition_until = 0.0
        self.static_label = None

    def update(self, now, raw, pose=None, state='IDLE'):
        if raw in {'NO_HAND', 'CAMERA_ERROR', 'MULTIPLE_HANDS', 'STALE_RESULT'}:
            self.reset()
            return raw
        if state == 'RUNNING':
            self.reset()
            return 'Open_Palm' if raw == 'Open_Palm' or pose and pose.open else raw
        if raw != self.static_label:
            self.static_label, self.static_since = raw, now
        held = now-self.static_since
        pending = now < self.transition_until
        if raw == 'Thumb_Up' and (pending or state == 'SELECTED'):
            self.wave.reset()
            self.last_signal_at = now
            return raw
        if pose is None:
            if now-self.last_signal_at > self.wave.gap_s:
                self.reset()
            return raw
        self.last_signal_at = now
        static = raw in {'Thumb_Up', 'Pointing_Up', 'Victory', 'Closed_Fist'}
        # Brief static labels during finger flexion must not destroy motion
        # history. A deliberately held pose wins; stop while RUNNING is above.
        delay = .4 if raw == 'Closed_Fist' else .5
        if static and (not self.wave.collecting and not pending or held >= delay):
            self.wave.reset()
            self.transition_until = 0.0
            return raw
        if pending:
            return 'WAVE_PENDING'
        if self.wave.update(now, pose, open_hint=raw == 'Open_Palm'):
            self.transition_until = now+self.confirm_window_s
            return 'Wave'
        if raw == 'Open_Palm' or pose.open:
            return 'Open_Palm'
        return 'UNKNOWN'
