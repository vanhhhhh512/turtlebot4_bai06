"""Gesture selection, confirmation, odometry controlled motion and fail safe stops."""
import csv
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import DockStatus, HazardDetection, HazardDetectionVector
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String


CHOICES = {'Wave': 'GREET', 'Pointing_Up': 'FORWARD_BACKWARD_4',
           'Victory': 'CIRCLE'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        for name, value in [
            ('watchdog_s', 0.5), ('odom_timeout_s', 0.5), ('stable_s', 0.2),
            ('linear_speed', 0.12), ('angular_speed', 0.42),
            ('leg_distance', 0.4), ('circle_radius', 0.6),
            ('arena_limit', 4.5), ('output_dir', ''),
            ('configure_sim_backup', False),
            ('require_sim_ready', False),
        ]:
            self.declare_parameter(name, value)
        self.cfg = {key: self.get_parameter(key).value for key in [
            'watchdog_s', 'odom_timeout_s', 'stable_s', 'linear_speed',
            'angular_speed', 'leg_distance', 'circle_radius', 'arena_limit']}
        self.cfg['linear_speed'] = min(max(float(self.cfg['linear_speed']), 0.0), 0.15)
        self.cfg['angular_speed'] = min(max(float(self.cfg['angular_speed']), 0.0), 0.5)
        self.cfg['circle_radius'] = max(float(self.cfg['circle_radius']), 0.4)
        self.output_dir = self.get_parameter('output_dir').value
        self.events = self.motion = self.writer = None
        if self.output_dir:
            os.makedirs(self.output_dir, exist_ok=True)
            self.events = open(os.path.join(self.output_dir, 'behavior.jsonl'), 'a', buffering=1)
            self.motion = open(os.path.join(self.output_dir, 'motion.csv'), 'a', newline='', buffering=1)
            self.writer = csv.writer(self.motion)
            if self.motion.tell() == 0:
                self.writer.writerow(['wall_s', 'monotonic_s', 'x_m', 'y_m', 'yaw_rad',
                                      'linear_x_mps', 'angular_z_radps', 'state', 'phase'])
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/behavior_status', 1)
        self.create_subscription(String, '/detected_gesture', self.on_gesture, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(DockStatus, '/dock_status', self.on_dock,
                                 qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, '/hazard_detection',
                                 self.on_hazard, qos_profile_sensor_data)
        self.state = 'IDLE'
        self.selected = None
        self.phase = ''
        self.phase_index = 0
        self.cycle = 0
        self.last_gesture_at = 0.0
        self.last_odom_at = 0.0
        self.odom = None
        self.docked = None
        self.hazard_active = False
        self.pose = None
        self.previous_yaw = None
        self.yaw_travel = 0.0
        self.phase_start_yaw_travel = 0.0
        self.phase_start_pose = None
        self.circle_heading = 0.0
        self.candidate = None
        self.candidate_since = 0.0
        self.candidate_count = 0
        self.candidate_last_at = 0.0
        self.fired = False
        self.last_label = None
        self.last_cmd = (0.0, 0.0)
        self.sim_ready = not self.get_parameter('require_sim_ready').value
        self.preparation_reason = 'WAITING_FOR_CONTROLLERS'
        self.create_subscription(String, '/simulation_ready', self.on_sim_ready, 1)
        self.sim_backup_ready = not self.get_parameter('configure_sim_backup').value
        self.sim_setup_future = None
        if not self.sim_backup_ready:
            self.sim_parameter_client = self.create_client(
                SetParameters, '/motion_control/set_parameters')
            self.create_timer(0.5, self.configure_sim_backup)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Waiting for fresh webcam gestures and odometry')

    def configure_sim_backup(self):
        # Enabled only by this package's Gazebo launch. Create 3 otherwise
        # exhausts its short backup buffer before a 0.40 m reverse leg.
        if self.sim_backup_ready or self.sim_setup_future is not None:
            return
        if not self.sim_parameter_client.service_is_ready():
            return
        request = SetParameters.Request(parameters=[Parameter(
            'safety_override', Parameter.Type.STRING, 'backup_only').to_parameter_msg()])
        self.sim_setup_future = self.sim_parameter_client.call_async(request)
        self.sim_setup_future.add_done_callback(self.on_sim_backup_configured)

    def on_sim_ready(self, msg):
        try:
            data = json.loads(msg.data)
            self.sim_ready = bool(data['ready'])
            self.preparation_reason = data.get('reason', 'PREPARING')
            if not self.sim_ready and self.state in {'SELECTED', 'RUNNING'}:
                self.stop('ROBOT_NOT_READY')
        except (ValueError, KeyError, TypeError):
            pass

    def on_sim_backup_configured(self, future):
        try:
            response = future.result()
            self.sim_backup_ready = bool(response.results) and all(
                result.successful for result in response.results)
            if self.sim_backup_ready:
                self.event('SIM_BACKUP_READY')
            else:
                self.get_logger().warning('Gazebo backup configuration rejected; motion blocked')
        except Exception as exc:
            self.get_logger().warning(f'Gazebo backup configuration failed: {exc}')
        finally:
            self.sim_setup_future = None

    def event(self, reason):
        record = dict(wall=time.time(), monotonic=time.monotonic(),
                      state=self.state, selected=self.selected, phase=self.phase,
                      cycle=self.cycle, reason=reason)
        self.get_logger().info(json.dumps(record))
        if self.events:
            self.events.write(json.dumps(record) + '\n')
        self.publish_status(reason)

    def publish_status(self, reason=''):
        ready = self.sim_ready and self.docked is False and self.sim_backup_ready
        blocked = (self.preparation_reason if not self.sim_ready else
                   'WAITING_FOR_UNDOCK' if self.docked is not False else
                   'CONFIGURING_SIMULATION' if not self.sim_backup_ready else '')
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.state, selected=self.selected, phase=self.phase,
            cycle=self.cycle, reason=reason, sim_backup_ready=self.sim_backup_ready,
            ready=ready, blocked_reason=blocked))))

    def on_odom(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        if self.previous_yaw is not None:
            self.yaw_travel += angle_delta(yaw, self.previous_yaw)
        self.previous_yaw = yaw
        self.pose = (p.x, p.y, yaw)
        self.odom = msg
        self.last_odom_at = now
        if self.state == 'RUNNING' and (
            abs(p.x) > self.cfg['arena_limit'] or abs(p.y) > self.cfg['arena_limit']
        ):
            self.stop('ARENA_LIMIT')

    def on_dock(self, msg):
        self.docked = bool(msg.is_docked)
        if self.docked and self.state == 'RUNNING':
            self.stop('DOCKED')

    def on_hazard(self, msg):
        unsafe = {HazardDetection.BUMP, HazardDetection.CLIFF,
                  HazardDetection.STALL, HazardDetection.WHEEL_DROP}
        self.hazard_active = any(d.type in unsafe for d in msg.detections)
        if self.state == 'RUNNING' and self.hazard_active:
            self.stop('HAZARD')

    def on_gesture(self, msg):
        now = time.monotonic()
        label = msg.data
        self.last_label = label
        if label == 'WAVE_PENDING':
            if self.state == 'SELECTED' and self.selected == 'GREET':
                self.last_gesture_at = now
            return
        if not self.sim_ready and (label in CHOICES or label == 'Thumb_Up'):
            self.send_cmd(0.0, 0.0)
            self.publish_status('ROBOT_NOT_READY')
            return
        if label in CHOICES or label in {'Thumb_Up', 'Open_Palm', 'Closed_Fist'}:
            self.last_gesture_at = now
        if label == 'Open_Palm':
            self.stop('OPEN_PALM')
            self.candidate = label
            self.fired = True
            return
        if label == 'Closed_Fist':
            self.stop('CLOSED_FIST')
            self.candidate = label
            self.fired = True
            return
        if label in {'NO_HAND', 'CAMERA_ERROR', 'MULTIPLE_HANDS', 'STALE_RESULT'}:
            self.stop(label)
            self.candidate = label
            self.fired = False
            return
        if label == 'UNKNOWN':
            # A single uncertain frame while changing pose must not discard
            # valid confirmation samples. It never refreshes the watchdog.
            if now - self.candidate_last_at > 0.18:
                self.candidate = None
                self.candidate_count = 0
                self.fired = False
            return
        if label != self.candidate or now - self.candidate_last_at > 0.18:
            self.candidate = label
            self.candidate_since = now
            self.candidate_count = 0
            self.fired = False
        self.candidate_count += 1
        self.candidate_last_at = now
        if self.fired or label not in CHOICES and label != 'Thumb_Up':
            return
        if label != 'Wave' and (now - self.candidate_since < self.cfg['stable_s']
                                or self.candidate_count < 3):
            return
        self.fired = True
        if self.state == 'RUNNING':
            return
        if label in CHOICES:
            self.selected = CHOICES[label]
            self.state = 'SELECTED'
            self.phase = ''
            self.event(f'SELECT_{label}')
        elif label == 'Thumb_Up' and self.state == 'SELECTED':
            if self.docked is not False:
                self.stop('NOT_UNDOCKED')
            elif not self.sim_backup_ready:
                self.stop('SIM_BACKUP_NOT_READY')
            elif self.hazard_active:
                self.stop('HAZARD_AT_CONFIRM')
            elif self.pose is None or now - self.last_odom_at > self.cfg['odom_timeout_s']:
                self.stop('NO_ODOM_AT_CONFIRM')
            elif max(abs(self.pose[0]), abs(self.pose[1])) > self.cfg['arena_limit']:
                self.stop('ARENA_LIMIT')
            else:
                self.start()

    def start(self):
        self.state = 'RUNNING'
        self.cycle = 0
        self.phase_index = 0
        self.phase_start_pose = self.pose
        self.phase_start_yaw_travel = self.yaw_travel
        self.phase = {'GREET': 'TURN_LEFT', 'FORWARD_BACKWARD_4': 'FORWARD',
                      'CIRCLE': 'ALIGN_CIRCLE'}[self.selected]
        if self.selected == 'CIRCLE':
            # The dock is near the arena origin. Align the tangent so the
            # circle's centre lies farther from it, rather than curving back
            # towards the dock immediately after undocking.
            x, y, yaw = self.pose
            self.circle_heading = (math.atan2(y, x) - math.pi / 2
                                   if math.hypot(x, y) > 0.1 else yaw)
        self.event('CONFIRMED')

    def stop(self, reason):
        changed = self.state != 'IDLE' or self.selected is not None
        self.state = 'IDLE'
        self.selected = None
        self.phase = ''
        self.send_cmd(0.0, 0.0)
        if changed:
            self.event(reason)
        self.cycle = 0

    def phase_distance(self):
        x0, y0, yaw0 = self.phase_start_pose
        x, y, _ = self.pose
        return (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)

    def next_phase(self):
        self.phase_start_pose = self.pose
        self.phase_start_yaw_travel = self.yaw_travel
        if self.selected == 'GREET':
            self.phase_index += 1
            phases = ['TURN_LEFT', 'TURN_RIGHT', 'RETURN_CENTER']
            if self.phase_index >= len(phases):
                self.stop('COMPLETED_GREET')
            else:
                self.phase = phases[self.phase_index]
                self.event('NEXT_PHASE')
        elif self.selected == 'FORWARD_BACKWARD_4':
            if self.phase == 'FORWARD':
                self.phase = 'BACKWARD'
                self.event('NEXT_PHASE')
            else:
                self.cycle += 1
                if self.cycle >= 4:
                    self.stop('COMPLETED_FORWARD_BACKWARD_4')
                else:
                    self.phase = 'FORWARD'
                    self.event('NEXT_PHASE')

    def tick(self):
        now = time.monotonic()
        if self.state in {'SELECTED', 'RUNNING'} and (
            now - self.last_gesture_at > self.cfg['watchdog_s']
        ):
            self.stop('GESTURE_TIMEOUT')
        if self.docked is not False:
            self.publish_status('WAITING_FOR_UNDOCK')
            return
        if self.state == 'RUNNING':
            if now - self.last_odom_at > self.cfg['odom_timeout_s']:
                self.stop('ODOM_TIMEOUT')
        linear = angular = 0.0
        if self.state == 'RUNNING':
            if self.selected == 'GREET':
                targets = [0.45, -0.90, 0.45]
                target = targets[self.phase_index]
                turned = self.yaw_travel - self.phase_start_yaw_travel
                if turned * math.copysign(1.0, target) >= abs(target):
                    self.next_phase()
                else:
                    angular = math.copysign(self.cfg['angular_speed'], target)
            elif self.selected == 'FORWARD_BACKWARD_4':
                distance = self.phase_distance()
                target = self.cfg['leg_distance']
                direction = 1 if self.phase == 'FORWARD' else -1
                if distance * direction >= target:
                    self.next_phase()
                else:
                    linear = self.cfg['linear_speed'] * direction
            elif self.selected == 'CIRCLE':
                if self.phase == 'ALIGN_CIRCLE':
                    error = angle_delta(self.circle_heading, self.pose[2])
                    if abs(error) <= 0.03:
                        self.phase = 'CIRCLE'
                        self.phase_start_yaw_travel = self.yaw_travel
                        self.event('NEXT_PHASE')
                    else:
                        angular = math.copysign(self.cfg['angular_speed'], error)
                elif self.yaw_travel - self.phase_start_yaw_travel >= 2 * math.pi:
                    self.stop('COMPLETED_CIRCLE')
                else:
                    linear = self.cfg['linear_speed']
                    angular = min(linear / self.cfg['circle_radius'], self.cfg['angular_speed'])
        self.send_cmd(linear, angular)
        if self.pose and self.writer:
            self.writer.writerow([time.time(), now, *self.pose, linear, angular,
                                  self.state, self.phase])
        self.publish_status()

    def send_cmd(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = max(-0.15, min(0.15, linear))
        cmd.angular.z = max(-0.5, min(0.5, angular))
        self.cmd_pub.publish(cmd)
        self.last_cmd = (cmd.linear.x, cmd.angular.z)

    def close(self):
        if rclpy.ok():
            for _ in range(5):
                self.send_cmd(0.0, 0.0)
        if self.events:
            self.events.close()
        if self.motion:
            self.motion.close()


def main():
    rclpy.init()
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
