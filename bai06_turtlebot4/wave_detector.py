"""Detect a left-right wave within a bounded window of fresh landmarks."""
from collections import deque
import math


def open_hand_motion(landmarks):
    """Return a motion point only for an extended, open hand.

    Distances and joint angles work for either hand, including a tilted wrist.
    Tracking both palm and fingertips also captures a wrist wave in which the
    palm centre barely moves. This is geometry, not a model confidence score.
    """
    if len(landmarks) != 21:
        return None

    def distance(a, b):
        return math.hypot(a.x - b.x, a.y - b.y)

    wrist = landmarks[0]
    extended = 0
    for mcp, pip, tip in [(5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)]:
        a, b, c = (landmarks[i] for i in (mcp, pip, tip))
        ux, uy, vx, vy = a.x - b.x, a.y - b.y, c.x - b.x, c.y - b.y
        norm = math.hypot(ux, uy) * math.hypot(vx, vy)
        straight = norm > 1e-8 and (ux * vx + uy * vy) / norm < -0.75
        if straight and distance(c, wrist) > 1.12 * distance(b, wrist):
            extended += 1
    if extended < 3:
        return None
    width = distance(landmarks[5], landmarks[17])
    if width < 0.025:
        return None
    palm_x = sum(landmarks[i].x for i in (0, 5, 9, 17)) / 4
    palm_y = sum(landmarks[i].y for i in (0, 5, 9, 17)) / 4
    tips_x = sum(landmarks[i].x for i in (8, 12, 16, 20)) / 4
    tips_y = sum(landmarks[i].y for i in (8, 12, 16, 20)) / 4
    return ((palm_x + tips_x) / 2, (palm_y + tips_y) / 2, width)


class WaveDetector:
    def __init__(self, window_s=2.4, min_travel=0.025, min_reversals=2,
                 hand_scale=0.30, gap_s=0.20):
        self.window_s = window_s
        self.min_travel = min_travel
        self.min_reversals = min_reversals
        self.hand_scale = hand_scale
        self.gap_s = gap_s
        self.samples = deque()
        self.last_emit = -1e9
        self.reversals = 0
        self.threshold = min_travel

    def reset(self):
        self.samples.clear()
        self.reversals = 0

    def update(self, now, palm_x, palm_y, open_palm, hand_width=None):
        if not open_palm:
            self.reset()
            return False
        if self.samples and now - self.samples[-1][0] > self.gap_s:
            self.reset()
        self.samples.append((now, palm_x, palm_y))
        while self.samples and now - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        # Scale with the hand, rather than demanding a fixed 54-pixel stroke.
        self.threshold = max(self.min_travel, min(0.07,
            hand_width * self.hand_scale if hand_width is not None else self.min_travel))
        self.reversals = 0
        if len(self.samples) < 4:
            return False
        _, extreme_x, extreme_y = self.samples[0]
        direction = reversals = 0
        for _, x, y in self.samples:
            # Keep the actual turning point. Resetting the anchor after every
            # threshold crossing misses a shorter return stroke near a peak.
            if direction and (x - extreme_x) * direction > 0:
                extreme_x, extreme_y = x, y
                continue
            dx, dy = x - extreme_x, y - extreme_y
            if abs(dx) < self.threshold or abs(dx) <= abs(dy) * 1.2:
                continue
            new_direction = 1 if dx > 0 else -1
            if direction and new_direction != direction:
                reversals += 1
            direction = new_direction
            extreme_x, extreme_y = x, y
        self.reversals = reversals
        if (reversals >= self.min_reversals
                and now - self.samples[0][0] >= 0.30
                and now - self.last_emit > 1.2):
            self.last_emit = now
            self.reset()
            return True
        return False


class GestureRouter:
    """Open Palm always passes through while RUNNING.

    After a Wave event, ignore only the same open-hand transition, for at
    most 0.6 seconds. A palm held open after that is STOP. Any other pose
    ends the transition immediately. The Wave label remains visible only on
    fresh open-hand frames during this bounded handoff; it never masks a stop
    while RUNNING or a missing/stale/multiple-hand result.
    """
    def __init__(self, **wave_options):
        self.wave = WaveDetector(**wave_options)
        self.transition_until = 0.0

    def update(self, now, raw, palm=None, state='IDLE'):
        signal_lost = raw in {'NO_HAND', 'CAMERA_ERROR', 'MULTIPLE_HANDS', 'STALE_RESULT'}
        open_candidate = palm is not None and raw in {'Open_Palm', 'UNKNOWN'}
        if state == 'RUNNING':
            self.wave.reset()
            self.transition_until = 0.0
            return 'Open_Palm' if open_candidate else raw
        if signal_lost or raw in {'Closed_Fist', 'Thumb_Up', 'Pointing_Up', 'Victory'}:
            self.wave.reset()
            self.transition_until = 0.0
            return raw
        if not open_candidate:
            # A brief unclassified pose may bridge samples, but never supplies
            # a sample, an event or a heartbeat. Actual tracking loss clears it.
            if self.wave.samples and now - self.wave.samples[-1][0] > self.wave.gap_s:
                self.wave.reset()
            self.transition_until = 0.0
            return raw
        if now < self.transition_until:
            return 'Wave'
        width = palm[2] if len(palm) > 2 else None
        if self.wave.update(now, palm[0], palm[1], True, width):
            self.transition_until = now + 0.6
            return 'Wave'
        return 'Open_Palm'
