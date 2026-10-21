"""Detect a left-right wave within a bounded window of fresh landmarks."""
from collections import deque


class WaveDetector:
    def __init__(self, window_s=1.6, min_travel=0.085, min_reversals=2):
        self.window_s = window_s
        self.min_travel = min_travel
        self.min_reversals = min_reversals
        self.samples = deque()
        self.last_emit = -1e9

    def reset(self):
        self.samples.clear()

    def update(self, now, palm_x, palm_y, open_palm):
        if not open_palm:
            self.reset()
            return False
        self.samples.append((now, palm_x, palm_y))
        while self.samples and now - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        # Recompute from samples still in the window: old reversals cannot
        # leak into a later, unrelated open-palm motion.
        if len(self.samples) < 4:
            return False
        _, anchor_x, anchor_y = self.samples[0]
        direction = reversals = 0
        for _, x, y in self.samples:
            dx, dy = x - anchor_x, y - anchor_y
            if abs(dx) < self.min_travel or abs(dx) <= abs(dy) * 1.3:
                continue
            new_direction = 1 if dx > 0 else -1
            if direction and new_direction != direction:
                reversals += 1
            direction = new_direction
            anchor_x, anchor_y = x, y
        if reversals >= self.min_reversals and now - self.last_emit > 1.2:
            self.last_emit = now
            self.reset()
            return True
        return False


class GestureRouter:
    """Open Palm always passes through while RUNNING.

    After a Wave event, ignore only the same open-hand transition, for at
    most 0.6 seconds. A palm held open after that is STOP. Any other pose
    ends the transition immediately.
    """
    def __init__(self):
        self.wave = WaveDetector()
        self.transition_until = 0.0

    def update(self, now, raw, palm=None, state='IDLE'):
        if raw != 'Open_Palm' or palm is None:
            self.wave.reset()
            self.transition_until = 0.0
            return raw
        if state == 'RUNNING':
            self.wave.reset()
            self.transition_until = 0.0
            return 'Open_Palm'
        if now < self.transition_until:
            return 'UNKNOWN'
        if self.wave.update(now, *palm, True):
            self.transition_until = now + 0.6
            return 'Wave'
        return 'Open_Palm'
