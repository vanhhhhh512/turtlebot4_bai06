"""Filter fresh static predictions without replaying a previous gesture."""

STATIC_GESTURES = {'Pointing_Up', 'Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist'}


class StaticGestureFilter:
    def __init__(self, confidence=.60, thumb_confidence=.55, thumb_hold_confidence=.45):
        self.confidence = confidence
        self.thumb_enter = min(confidence, thumb_confidence)
        self.thumb_hold = min(self.thumb_enter, thumb_hold_confidence)
        self.reset()

    def reset(self):
        self.thumb_active = False

    def update(self, category, score, pose=None):
        if category != 'Thumb_Up':
            self.reset()
            return category if category in STATIC_GESTURES and score >= self.confidence else 'UNKNOWN'
        # Below the general threshold, require an extended thumb and folded
        # fingers as well as a fresh Thumb_Up prediction from the model.
        thumb_shape = (pose is not None and pose.thumb_open
                       and sum(v <= .45 for v in pose.extension) >= 3
                       and max(pose.extension) < .65)
        threshold = self.thumb_hold if self.thumb_active else self.thumb_enter
        accepted = score >= self.confidence or thumb_shape and score >= threshold
        self.thumb_active = bool(accepted)
        return 'Thumb_Up' if accepted else 'UNKNOWN'
