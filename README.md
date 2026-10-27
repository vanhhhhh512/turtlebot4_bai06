# Bài thực hành 06 — TurtleBot 4 và điều khiển bằng cử chỉ tay

Chương trình dùng webcam laptop, OpenCV và MediaPipe Gesture Recognizer để nhận
cử chỉ tay, sau đó điều khiển TurtleBot 4 trong Gazebo qua ROS 2.
Hai node chính bằng Python chạy độc lập: node nhận diện `/detected_gesture`, node hành vi
quản lý chọn, xác nhận `/cmd_vel`.

## Cài đặt

Cần Ubuntu 22.04, ROS 2 Humble và TurtleBot 4 Simulator đã cấu hình repository apt theo
[hướng dẫn TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html).

```bash
sudo apt update
sudo apt install xdotool ros-humble-turtlebot4-simulator \
  build-essential pkg-config python3-dev libignition-transport11-dev libignition-msgs8-dev \
  python3-colcon-common-extensions python3-pip python3-opencv python3-numpy
mkdir -p ~/turtlebot6_github_ws/src
cd ~/turtlebot6_github_ws/src
git clone https://github.com/vanhhhhh512/turtlebot4_bai06.git bai06_turtlebot4
cd ~/turtlebot6_github_ws
python3 -m pip install --target python_deps \
  'mediapipe==0.10.21' 'opencv-contrib-python==4.11.0.86' \
  'numpy==1.26.4' 'protobuf==4.25.9'
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/bai06_turtlebot4 \
  --packages-select bai06_turtlebot4
source install/setup.bash
```

Model `models/gesture_recognizer.task` được đặt sẵn trong repo. Các thư viện Python
được cài vào thư mục riêng của workspace. Dùng Python 3.10.12,
OpenCV 4.11.0, NumPy 1.26.4 và MediaPipe 0.10.21.

## Chạy toàn bộ bằng một lệnh

```bash
cd ~/turtlebot6_github_ws/src/bai06_turtlebot4
./scripts/run_demo.sh
```

Script build package, tạo thư mục output mới, ROS domain và Gazebo partition
riêng; mở Gazebo, TurtleBot 4, webcam và hai node. Robot thực hiện action `/undock`
trong bước chuẩn bị rồi về chờ. Chỉ chọn cử chỉ khi robot đã ra khỏi dock.

| Cử chỉ | Hành vi |
|---|---|
| Wave: xòe 5 ngón rồi vẫy nhẹ các ngón hai lần | Chọn chào trái–phải rồi về hướng gần ban đầu |
| Pointing Up: giơ ngón trỏ | Chọn tiến–lùi 4 chu kỳ, mỗi chặng 0,40 m |
| Victory: giơ hai ngón chữ V | Chọn đi tròn một vòng, bán kính 0,60 m |
| Thumbs Up: giơ ngón cái | Xác nhận chạy; khi PAUSED thì tiếp tục phần còn lại |
| Open Palm: xòe cả bàn tay | Tạm dừng ngay, giữ hành vi và tiến độ (PAUSED) |
| Closed Fist: nắm tay | Hủy hành vi và toàn bộ tiến độ, trở về chờ |

Mọi chuyển động, kể cả chào, đều cần chọn rồi xác nhận. **Sau khi Thumb Up
xác nhận, có thể bỏ tay khỏi camera; robot tự chạy hết tác vụ theo odometry.**

**Open Palm → PAUSED:** vận tốc về 0 ngay, giữ pha, chu kỳ và tiến độ chặng.
Thumb Up xác nhận để tiếp tục phần còn lại. Ví dụ đang lùi ở lượt 3/4 thì tiếp
tục chặng lùi đó, không quay lại lượt 1. Có thể bỏ tay ra trong lúc PAUSED;
robot đứng yên đến khi nhận Thumb Up. **Closed Fist → IDLE:** xóa lựa chọn và
tiến độ; muốn chạy cần chọn lại rồi xác nhận.

## Tài liệu tham khảo

- [MediaPipe Gesture Recognizer](https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer)
- [MediaPipe Gesture Recognizer Python](https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer/python)
- [OpenCV VideoCapture](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)
- [TurtleBot 4 Simulation](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html)
