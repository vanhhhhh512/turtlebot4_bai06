"""Recognise a finger wave: extend, curl, extend twice before confirming."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FingerPose:
    extension: tuple

    @property
    def open(self):
        return sum(value >= 0.70 for value in self.extension) >= 3

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
    return FingerPose(tuple(values))


class FingerWaveDetector:
    def __init__(self, window_s=3.0, min_cycles=2, hold_s=0.07, gap_s=0.25):
        self.window_s = window_s
        self.min_cycles = min_cycles
        self.hold_s = hold_s
        self.gap_s = gap_s
        self.last_emit = -1e9
        self.reset()

    def reset(self):
        self.phase = ''
        self.candidate = ''
        self.candidate_at = 0.0
        self.candidate_count = 0
        self.started_at = None
        self.last_at = -1e9
        self.cycles = 0

    @property
    def collecting(self):
        return bool(self.phase)

    def update(self, now, pose):
        if (now - self.last_at > self.gap_s
                or self.started_at is not None and now - self.started_at > self.window_s):
            self.reset()
        self.last_at = now
        level = 'OPEN' if pose.open else 'CURL' if pose.curled else ''
        if not level:
            self.candidate = ''
            return False
        if level != self.candidate:
            self.candidate = level
            self.candidate_at = now
            self.candidate_count = 0
        self.candidate_count += 1
        if self.candidate_count < 2 or now - self.candidate_at < self.hold_s:
            return False
        if not self.phase:
            if level == 'OPEN':
                self.phase = 'OPEN'
            return False
        if self.phase == 'OPEN' and level == 'CURL':
            if self.started_at is None:
                self.started_at = now
            self.phase = 'CURL'
        elif self.phase == 'CURL' and level == 'OPEN':
            self.phase = 'OPEN'
            self.cycles += 1
            if self.cycles >= self.min_cycles and now - self.last_emit > 1.0:
                self.last_emit = now
                self.reset()
                return True
        return False


class GestureRouter:
    """Separate finger motion, confirmation, cancellation, and immediate stops."""
    def __init__(self, confirm_window_s=3.0, **wave_options):
        self.wave = FingerWaveDetector(**wave_options)
        self.confirm_window_s = min(max(float(confirm_window_s), 0.5), 3.0)
        self.transition_until = 0.0
        self.last_signal_at = -1e9
        self.fist_since = None

    def reset(self):
        self.wave.reset()
        self.transition_until = 0.0
        self.fist_since = None

    def update(self, now, raw, pose=None, state='IDLE'):
        if raw in {'NO_HAND', 'CAMERA_ERROR', 'MULTIPLE_HANDS', 'STALE_RESULT'}:
            self.reset()
            return raw
        if state == 'RUNNING':
            self.reset()
            return 'Open_Palm' if raw == 'Open_Palm' or pose and pose.open else raw
        if raw in {'Thumb_Up', 'Pointing_Up', 'Victory'}:
            self.wave.reset()
            self.fist_since = None
            self.last_signal_at = now
            if raw != 'Thumb_Up':
                self.transition_until = 0.0
            return raw
        if raw == 'Closed_Fist':
            if self.fist_since is None:
                self.fist_since = now
            # Transient curl is part of a wave. Holding a fist cancels after
            # 0.4 s while stationary; RUNNING always passes cancel immediately.
            if (not self.wave.collecting and now >= self.transition_until
                    or now - self.fist_since >= 0.4):
                self.reset()
                return 'Closed_Fist'
        else:
            self.fist_since = None
        if pose is None:
            if now - self.last_signal_at > self.wave.gap_s:
                self.reset()
            return 'UNKNOWN'
        self.last_signal_at = now
        if now < self.transition_until:
            return 'WAVE_PENDING'
        if self.wave.update(now, pose):
            self.transition_until = now + self.confirm_window_s
            return 'Wave'
        if raw == 'Open_Palm' or pose.open:
            return 'Open_Palm'
        return 'UNKNOWN'
