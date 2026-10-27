"""Recognise an open-hand finger flutter before gesture confirmation."""
from collections import deque
from dataclasses import dataclass
from statistics import median
import math


@dataclass(frozen=True)
class FingerPose:
    extension: tuple
    thumb_open: bool = True
    motion: tuple = ()

    @property
    def open(self):
        return self.thumb_open and sum(value >= 0.70 for value in self.extension) >= 3

    @property
    def curled(self):
        return sum(value <= 0.40 for value in self.extension) >= 3


def finger_pose(landmarks, y_scale=1.0):
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
    # Express each tip relative to its MCP in a palm-attached 3D frame.
    # This retains side/depth wiggles while a straight finger's distance ratio
    # is saturated. Translation, rotation and uniform scale cancel out.
    xyz = [(p.x, p.y*y_scale, getattr(p, 'z', 0)) for p in landmarks]
    def sub(a, b):
        return tuple(x-y for x,y in zip(a,b))
    def dot(a, b):
        return sum(x*y for x,y in zip(a,b))
    def length(a):
        return math.sqrt(dot(a,a))
    forward = sub(xyz[9], xyz[0])
    size = length(forward)
    if size < 1e-5:
        return None
    forward = tuple(v/size for v in forward)
    side = sub(xyz[5], xyz[17])
    projection = dot(side, forward)
    side = tuple(v-projection*y for v,y in zip(side,forward))
    width = length(side)
    if width < 1e-5:
        return None
    side = tuple(v/width for v in side)
    normal = (side[1]*forward[2]-side[2]*forward[1],
              side[2]*forward[0]-side[0]*forward[2],
              side[0]*forward[1]-side[1]*forward[0])
    motion = []
    for base in (5,9,13,17):
        bone_length = sum(length(sub(xyz[i+1],xyz[i])) for i in range(base,base+3))
        if bone_length < 1e-5:
            return None
        tip = sub(xyz[base+3],xyz[base])
        motion.append(tuple(dot(tip,axis)/bone_length for axis in (side,forward,normal)))
    return FingerPose(tuple(values), thumb_open, tuple(motion))


class _FingerOscillation:
    """Count out-and-back strokes along this finger's own movement axis."""
    def __init__(self, anchor, swing):
        self.anchor = anchor
        self.swing = swing
        self.direction = None
        self.extreme = 0.
        self.outward = False
        self.cycles = 0

    def update(self, vector):
        delta = tuple(v-a for v,a in zip(vector,self.anchor))
        if self.direction is None:
            distance = math.sqrt(sum(v*v for v in delta))
            if distance >= self.swing:
                self.direction = tuple(v/distance for v in delta)
                self.extreme = distance
                self.outward = True
            return
        position = sum(v*d for v,d in zip(delta,self.direction))
        if self.outward:
            self.extreme = max(self.extreme,position)
            if self.extreme-position >= .75*self.swing:
                self.cycles += 1
                self.outward = False
                self.extreme = position
        else:
            self.extreme = min(self.extreme,position)
            if position-self.extreme >= self.swing:
                self.outward = True
                self.extreme = position


class FingerWaveDetector:
    """Arm with an open hand, then detect independent finger oscillations.

    Palm-local tip vectors capture lateral wiggles and flexion. A median of
    three fresh frames rejects spikes; three fingers must each repeat twice.
    """
    def __init__(self, window_s=4.0, min_cycles=2, min_swing=0.18, gap_s=0.25):
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
        self.fingers = []
        self.finger_cycles = [0]*4
        self.arm_count = 0
        self.vector_mode = None

    @property
    def collecting(self):
        return bool(self.phase)

    @property
    def hint(self):
        return 'Wiggle fingers gently' if self.phase else 'Open 5 fingers first'

    def update(self, now, pose, open_hint=False):
        vector_mode = bool(pose.motion)
        if (now-self.last_at > self.gap_s
                or self.started_at is not None and now-self.started_at > self.window_s
                or self.vector_mode is not None and vector_mode != self.vector_mode):
            self.reset()
        self.vector_mode = vector_mode
        self.last_at = now
        vectors = pose.motion if vector_mode else tuple((v,) for v in pose.extension)
        self.samples.append(vectors)
        smooth = tuple(tuple(median(sample[i][j] for sample in self.samples)
                             for j in range(len(vectors[i]))) for i in range(4))
        if not self.fingers:
            ready = (pose.thumb_open or open_hint) and sum(v >= .55 for v in pose.extension) >= 3
            self.arm_count = self.arm_count+1 if ready else 0
            if self.arm_count >= 2:
                swing = self.min_swing*.75 if vector_mode else self.min_swing
                self.fingers = [_FingerOscillation(v,swing) for v in smooth]
                self.phase = 'OPEN'
            return False
        for finger,vector in zip(self.fingers,smooth):
            finger.update(vector)
        if self.started_at is None and any(f.direction is not None for f in self.fingers):
            self.started_at = now
        self.finger_cycles = [f.cycles for f in self.fingers]
        self.cycles = sorted(self.finger_cycles,reverse=True)[2]
        self.phase = 'CURL' if sum(f.outward for f in self.fingers) >= 2 else 'OPEN'
        if (self.cycles >= self.min_cycles and self.started_at is not None
                and now-self.started_at >= .35 and now-self.last_emit > 1.0):
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
        if state in {'RUNNING', 'PAUSED'}:
            self.reset()
            # Hand tracking can briefly hallucinate extended fingers as a
            # Thumb Up leaves the image. Geometry alone is not a stop gesture:
            # require the fresh, confidence-filtered Open Palm prediction.
            # A valid stop still passes immediately, without Wave/debounce.
            return raw
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
        if raw == 'Open_Palm':
            return 'Open_Palm'
        return 'UNKNOWN'
