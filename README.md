# Bài thực hành 06 — TurtleBot 4 và điều khiển bằng cử chỉ tay

Chương trình dùng webcam laptop, OpenCV và MediaPipe Gesture Recognizer để nhận
cử chỉ tay, sau đó điều khiển TurtleBot 4 trong Gazebo qua ROS 2.
Hai node Python chạy độc lập: node nhận diện phát `/detected_gesture`, node hành vi
quản lý chọn, xác nhận và phát `/cmd_vel`.

## Cài đặt

Cần Ubuntu 22.04, ROS 2 Humble và TurtleBot 4 Simulator đã cấu hình repository apt theo
[hướng dẫn TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html).

```bash
sudo apt update
sudo apt install xdotool ros-humble-turtlebot4-simulator \
  python3-colcon-common-extensions python3-pip python3-opencv python3-numpy python3-pil
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
được cài vào thư mục riêng của workspace. Máy đã thử dùng Python 3.10.12,
OpenCV 4.11.0, NumPy 1.26.4 và MediaPipe 0.10.21.

## Chạy toàn bộ bằng một lệnh

Trong checkout Bài 06, chạy:

```bash
cd ~/turtlebot6_github_ws/src/bai06_turtlebot4
./scripts/run_demo.sh
```

Script build package, tạo thư mục output mới, ROS domain và Gazebo partition
riêng; mở Gazebo, TurtleBot 4, webcam và hai node. Robot thực hiện action `/undock`
trong bước chuẩn bị rồi về chờ. Chỉ chọn cử chỉ khi robot đã ra khỏi dock.
Launch Bài 06 cấu hình riêng controller Gazebo với `safety_override=backup_only`
để cho phép lùi hết chặng 0,40 m. Node chờ cấu hình thành công trước khi cho xác
nhận chạy; giới hạn tốc độ, vùng chạy, dừng/hủy và theo dõi hazard vẫn hoạt động.
Cấu hình này chỉ được bật bởi launch mô phỏng Bài 06.

Cửa sổ webcam hiển thị **21 điểm và đường nối bàn tay, nhãn, confidence, trạng thái
robot và số chu kỳ**. Ảnh được lật gương; phần chữ đặt ngoài ảnh để không che tay.
Gazebo và webcam được căn cạnh nhau để quan sát, quay video. Không dùng teleop
hoặc bàn phím phát vận tốc trong lúc demo. Nhấn Ctrl+C tại terminal chạy script
để đóng phiên.

| Cử chỉ | Hành vi |
|---|---|
| Wave: xòe tay, đưa qua lại ngang | Chọn chào trái–phải rồi về hướng gần ban đầu |
| Pointing Up: giơ ngón trỏ | Chọn tiến–lùi 4 chu kỳ, mỗi chặng 0,40 m |
| Victory: giơ hai ngón chữ V | Chọn đi tròn một vòng, bán kính 0,60 m |
| Thumbs Up: giơ ngón cái | Xác nhận và thực hiện lựa chọn đúng một lần |
| Open Palm: xòe cả bàn tay | Dừng ngay khi đang chạy, xóa lựa chọn |
| Closed Fist: nắm tay | Hủy hành vi, trở về chờ |

Mọi chuyển động, kể cả chào, đều cần chọn rồi xác nhận. Khi đang chạy, giữ một
cử chỉ hợp lệ trong ảnh để duy trì tín hiệu. Rút tay, camera lỗi, nhiều tay hoặc
mất kết quả hợp lệ quá 0,5 s làm robot dừng; muốn chạy tiếp phải chọn và xác nhận lại.

Pointing Up, Victory và Thumbs Up cần ít nhất 3 kết quả hợp lệ qua 0,2 s.
Một frame UNKNOWN xen giữa không xóa chuỗi nếu cách mẫu hợp lệ trước tối đa
0,18 s; UNKNOWN không gia hạn watchdog. Lệnh dừng/hủy không chờ bộ lọc này.
Giữ nguyên cử chỉ chỉ tạo một sự kiện; giữ Thumbs Up không kích hoạt lại sau khi
hành vi hoàn thành.

**Wave:** hướng bàn tay mở về camera, đưa qua lại trái–phải–trái trong khoảng
1–2 giây. Bộ nhận diện dùng cả tâm bàn tay và đầu ngón nên nhận được cả kiểu vẫy
bằng cổ tay. Cần hai lần đổi chiều trong 2,4 s; ngưỡng mỗi chặng bằng 30% bề rộng
bàn tay, tối thiểu 2,5% và tối đa 7% chiều rộng ảnh. Nhãn tĩnh `UNKNOWN` không
xóa chuỗi nếu landmarks vẫn cho thấy ít nhất ba ngón duỗi. Một khoảng không
phân loại được chỉ được nối tối đa 0,2 s; mất tay, nhiều tay hoặc kết quả quá cũ
xóa chuỗi ngay. Tay đứng yên và chuyển động dọc không được tính là Wave.

Cửa sổ hiện `turns 0/2`, `1/2` khi đang thu chuyển động. Khi nhận Wave sẽ hiện
`WAVE - GREET SELECTED` cùng bộ đếm **3 giây để đổi sang Thumbs Up**. Các frame
tay mở mới trong đoạn này giữ lựa chọn chào, không lặp sự kiện chọn. UNKNOWN
ngắn khi đổi tư thế không xóa bộ đếm; mất tay, nhiều tay, dữ liệu quá cũ hoặc
nắm tay hủy ngay. Hết 3 giây mà vẫn giữ tay mở thì xóa lựa chọn.

Khi robot đang chờ, cửa sổ ghi `OPEN HAND - wave left/right` để hướng dẫn chọn
Wave. **Khi robot đang chạy, Open Palm luôn dừng ngay**, kể cả đang còn bộ đếm
hoặc tay mở được xác định bằng landmarks khi model tĩnh trả về `UNKNOWN`.

Các ngưỡng Wave nằm trong `config/gestures.yaml`: `wave_window_s`,
`wave_min_travel`, `wave_hand_scale`, `wave_min_reversals`, `wave_confirm_window_s`.
Đoạn chuyển xác nhận được chặn tối đa 3 giây. Log `gestures.jsonl`
có thêm điểm chuyển động, ngưỡng và số lần đổi chiều để kiểm tra từng lượt.

Có thể đặt `BAI06_OUTPUT_DIR`, `BAI06_ROS_DOMAIN_ID`, `BAI06_MODEL`,
`BAI06_PYTHON_DEPS` hoặc `BAI06_UNDERLAY` trước lệnh chạy. Dùng
`BAI06_SKIP_BUILD=1 ./scripts/run_demo.sh` khi package đã build.
Máy phát triển dùng source tại `/home/admin1/xla/wk4/week6/ros2_ws/src/bai06_turtlebot4`
và tận dụng underlay TurtleBot 4 đã cài từ các bài trước.

## Source và dữ liệu ghi nhận

```text
bai06_turtlebot4/gesture_recognition_node.py  Webcam -> MediaPipe -> nhãn/khung tay
bai06_turtlebot4/wave_detector.py             Wave động và ưu tiên Open Palm
bai06_turtlebot4/visualization.py             Vẽ 21 landmark và đường nối
bai06_turtlebot4/robot_behavior_node.py       Chọn/xác nhận, odom, vận tốc, dừng
launch/demo.launch.py                       Gazebo, bridge, robot và hai node
config/gestures.yaml                        Ngưỡng, watchdog, tốc độ, quãng đường
worlds/gesture_arena.sdf                    Sàn trống 12 x 12 m
models/gesture_recognizer.task              Model MediaPipe chính thức
scripts/run_demo.sh                        Mở toàn bộ demo
scripts/arrange_windows.py                  Căn hai cửa sổ
scripts/record_screen.py                    Ghi màn hình thật
scripts/prepare_robot.sh                    Undock thủ công khi cần chuẩn bị lại
docs/report/                               PDF, source LaTeX, hình và ZIP Prism
```

Thư mục output gồm `gestures.jsonl`, `behavior.jsonl`, `motion.csv` và
`launch.log`. File gesture lưu nhãn/điểm/timestamp từng kết quả mới; file behavior
lưu chọn, xác nhận và lý do dừng. Motion CSV ghi pose odom cùng lệnh vận tốc ở timer
20 Hz. Thông số quãng đường và bán kính trong YAML là cấu hình điều khiển.

Node hành vi dùng odom kết thúc từng chặng và góc quay tích lũy để kết thúc một
vòng. Trước pha đi tròn, robot xoay hướng tại chỗ để tâm vòng nằm xa dock.
Vận tốc đầu ra được chặn ở 0,15 m/s và 0,5 rad/s. Mất odom, có bump/cliff/stall/
wheel drop hoặc ra khỏi vùng `|x|, |y| <= 4,5 m` làm dừng hành vi.

## Interface chính

`/detected_gesture` (`std_msgs/msg/String`), `/cmd_vel`
(`geometry_msgs/msg/Twist`), `/odom`, `/dock_status`, `/hazard_detection`, `/undock`.

Nhãn chuẩn là `Wave`, `Pointing_Up`, `Victory`, `Thumb_Up`, `Open_Palm`,
`Closed_Fist`; `Thumb_Up` là tên class thật của MediaPipe. `/gesture_status`
và `/behavior_status` dùng `std_msgs/msg/String` chứa JSON để hiển thị điểm nhận
diện, trạng thái, pha và chu kỳ. `NO_HAND`, `UNKNOWN`, `MULTIPLE_HANDS`,
`STALE_RESULT` thể hiện kết quả không cấp quyền chuyển động.

## Báo cáo và video demo

- [Báo cáo PDF](docs/report/Bao_cao_23020719_Bai_thuc_hanh_06.pdf)
- [Source LaTeX sửa trên Prism](docs/report/Bao_cao_23020719_Bai06_LaTeX_Prism.zip)
- [Video demo Google Drive](https://drive.google.com/drive/folders/1FdF8vADL3xgHH5QPhekMk_7wn1XFR4Jp?usp=drive_link)

## Tài liệu tham khảo

- [MediaPipe Gesture Recognizer](https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer)
- [MediaPipe Gesture Recognizer Python](https://developers.google.com/edge/mediapipe/solutions/vision/gesture_recognizer/python)
- [OpenCV VideoCapture](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)
- [TurtleBot 4 Simulation](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html)
