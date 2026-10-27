"""Draw the 21 MediaPipe landmarks and their anatomical connections."""
import cv2


HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)


def draw_hand_landmarks(frame, hands):
    height, width = frame.shape[:2]
    for landmarks in hands:
        if len(landmarks) != 21:
            continue
        points = [(round(p.x * (width - 1)), round(p.y * (height - 1)))
                  for p in landmarks]
        for a, b in HAND_CONNECTIONS:
            cv2.line(frame, points[a], points[b], (20, 220, 255), 2, cv2.LINE_AA)
        for index, point in enumerate(points):
            cv2.circle(frame, point, 5 if index == 0 else 4, (10, 10, 10), -1,
                       cv2.LINE_AA)
            cv2.circle(frame, point, 3, (80, 255, 80), -1, cv2.LINE_AA)
    return frame
