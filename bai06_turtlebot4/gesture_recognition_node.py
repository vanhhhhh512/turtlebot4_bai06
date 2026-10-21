"""Read the laptop webcam; publish only fresh MediaPipe results."""
import json
import os
import sys
import time

import cv2
# MediaPipe imports its optional audio module even for vision. On this machine
# sounddevice blocks during PortAudio discovery; no audio is used in this task.
sys.modules['sounddevice'] = None
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .wave_detector import GestureRouter
from .visualization import draw_hand_landmarks


VALID = {'Pointing_Up', 'Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist'}


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        for name, value in [
            ('device', 0), ('model_path', ''), ('confidence', 0.60),
            ('frame_width', 640), ('frame_height', 480),
            ('show_window', True), ('mirror', True), ('max_result_age_s', 0.25),
            ('output_dir', ''),
        ]:
            self.declare_parameter(name, value)
        self.output_dir = self.get_parameter('output_dir').value
        if self.output_dir:
            os.makedirs(self.output_dir, exist_ok=True)
            self.events = open(os.path.join(self.output_dir, 'gestures.jsonl'), 'a', buffering=1)
        else:
            self.events = None
        path = self.get_parameter('model_path').value
        if not os.path.isfile(path):
            raise FileNotFoundError(f'Gesture model missing: {path}')
        self.cap = cv2.VideoCapture(int(self.get_parameter('device').value), cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter('frame_width').value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter('frame_height').value))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError('Cannot open laptop webcam')
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=path),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=2,
        )
        self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        self.gesture_pub = self.create_publisher(String, '/detected_gesture', 1)
        self.status_pub = self.create_publisher(String, '/gesture_status', 1)
        self.create_subscription(String, '/behavior_status', self.on_behavior, 1)
        self.behavior = 'IDLE'
        self.behavior_state = 'IDLE'
        self.selected = None
        self.router = GestureRouter()
        self.last_timestamp_ms = -1
        self.seq = 0
        self.show = bool(self.get_parameter('show_window').value)
        self.threshold = float(self.get_parameter('confidence').value)
        self.mirror = bool(self.get_parameter('mirror').value)
        self.max_result_age_s = float(self.get_parameter('max_result_age_s').value)
        if self.show:
            cv2.namedWindow('Bai 06 - laptop webcam', cv2.WINDOW_NORMAL)
        self.get_logger().info('Webcam and MediaPipe model ready')

    def on_behavior(self, msg):
        try:
            data = json.loads(msg.data)
            self.behavior_state = data['state']
            self.selected = data.get('selected')
            self.behavior = f"{self.behavior_state} / {self.selected or '-'}"
            if data.get('phase'):
                self.behavior += f" / {data['phase']}"
            if self.selected == 'FORWARD_BACKWARD_4' and self.behavior_state == 'RUNNING':
                self.behavior += f" {min(data.get('cycle', 0) + 1, 4)}/4"
        except (KeyError, ValueError):
            pass

    def process_one(self):
        ok, frame = self.cap.read()
        now = time.monotonic()
        if not ok:
            self.publish('NO_HAND', 0.0, 'CAMERA_ERROR', now)
            self.router.update(now, 'NO_HAND')
            time.sleep(0.05)
            return
        if self.mirror:
            frame = cv2.flip(frame, 1)
        timestamp_ms = max(self.last_timestamp_ms + 1, int(now * 1000))
        self.last_timestamp_ms = timestamp_ms
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.recognizer.recognize_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp_ms)
        raw = 'NO_HAND'
        confidence = 0.0
        if len(result.hand_landmarks) == 1:
            raw = 'UNKNOWN'
            if result.gestures and result.gestures[0]:
                category = result.gestures[0][0]
                confidence = float(category.score)
                if category.category_name in VALID and confidence >= self.threshold:
                    raw = category.category_name
        elif len(result.hand_landmarks) > 1:
            raw = 'MULTIPLE_HANDS'
        palm = None
        if raw == 'Open_Palm':
            landmarks = result.hand_landmarks[0]
            palm_x = (landmarks[0].x + landmarks[5].x + landmarks[9].x + landmarks[17].x) / 4
            palm_y = (landmarks[0].y + landmarks[5].y + landmarks[9].y + landmarks[17].y) / 4
            palm = (palm_x, palm_y)
        if time.monotonic() - now > self.max_result_age_s:
            raw = 'STALE_RESULT'
            palm = None
        label = self.router.update(now, raw, palm, self.behavior_state)
        self.publish(label, confidence, raw, now)
        if self.show:
            draw_hand_landmarks(frame, result.hand_landmarks)
            display = cv2.copyMakeBorder(frame, 105, 65, 0, 0,
                                         cv2.BORDER_CONSTANT, value=(20, 20, 20))
            score = 'motion rule' if label == 'Wave' else f'model {confidence:.2f}'
            cv2.putText(display, f'{label} | {score}', (12, 29),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (70, 255, 100), 2)
            cv2.putText(display, self.behavior, (12, 57),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (240, 240, 240), 1)
            hint = ('THUMB UP to confirm' if self.behavior_state == 'SELECTED'
                    else 'Keep one hand visible' if self.behavior_state == 'RUNNING'
                    else 'Wave / 1 finger / 2 fingers: select, then THUMB UP')
            cv2.putText(display, hint, (12, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.47, (0, 220, 255), 1)
            y = display.shape[0] - 39
            cv2.putText(display, 'OPEN PALM: STOP | CLOSED FIST: CANCEL', (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.51, (230, 230, 230), 1)
            cv2.putText(display, 'Lost hand / signal: stop. Select + confirm to restart.',
                        (12, y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 180, 180), 1)
            cv2.imshow('Bai 06 - laptop webcam', display)
            cv2.waitKey(1)

    def publish(self, label, confidence, raw, now):
        self.seq += 1
        if not rclpy.ok():
            return
        self.gesture_pub.publish(String(data=label))
        record = dict(seq=self.seq, monotonic=now, wall=time.time(),
                      label=label, raw=raw, confidence=confidence)
        self.status_pub.publish(String(data=json.dumps(record)))
        if self.events:
            self.events.write(json.dumps(record) + '\n')

    def close(self):
        if rclpy.ok():
            self.publish('NO_HAND', 0.0, 'SHUTDOWN', time.monotonic())
        self.recognizer.close()
        self.cap.release()
        if self.show:
            cv2.destroyAllWindows()
        if self.events:
            self.events.close()


def main():
    rclpy.init()
    node = None
    try:
        node = GestureRecognitionNode()
        while rclpy.ok():
            node.process_one()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
