"""Draw a pose-following box and the travelled path in Gazebo's GUI only."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from . import _ign_markers


def point_text(point):
    return 'point { x: %.6f y: %.6f z: %.6f }' % tuple(point)


def box_edges(size):
    x,y,z = size
    vertices = [(a*x/2,b*y/2,c*z+.008) for c in (0,1) for b in (-1,1) for a in (-1,1)]
    edges = [(0,1),(1,3),(3,2),(2,0),(4,5),(5,7),(7,6),(6,4),(0,4),(1,5),(2,6),(3,7)]
    return [vertices[i] for edge in edges for i in edge]


def marker(marker_id,kind,points,color,pose=None,lifetime=0):
    transform = ('pose { position { x: %.6f y: %.6f z: %.6f } '
                 'orientation { x: %.8f y: %.8f z: %.8f w: %.8f } }' % pose
                 if pose else 'pose { orientation { w: 1 } }')
    rgba = 'r: %.3f g: %.3f b: %.3f a: 1' % color
    # Keep scale=1: vertices are already metres, not a pen-width setting.
    return ('marker { ns: "bai06_visualization" id: %d action: ADD_MODIFY '
            'type: %s visibility: GUI %s lifetime { sec: %d } '
            'scale { x: 1 y: 1 z: 1 } '
            'material { ambient { %s } diffuse { %s } emissive { %s } lighting: false } %s }'
            % (marker_id,kind,transform,lifetime,rgba,rgba,rgba,' '.join(map(point_text,points))))


class GazeboVisualizationNode(Node):
    def __init__(self):
        super().__init__('gazebo_visualization_node')
        for key,value in [('robot_name','turtlebot4'),('update_hz',10.),
                          ('trail_spacing_m',.015),('max_trail_points',2000),
                          ('box_size_x',.38),('box_size_y',.38),('box_size_z',.38),
                          ('shutdown_on_pose_loss_s',0.)]:
            self.declare_parameter(key,value)
        self.robot_name = self.get_parameter('robot_name').value
        self.spacing = max(.005,float(self.get_parameter('trail_spacing_m').value))
        self.points = deque(maxlen=max(2,int(self.get_parameter('max_trail_points').value)))
        self.edges = box_edges(tuple(max(.05,float(self.get_parameter('box_size_'+axis).value))
                                    for axis in ('x','y','z')))
        self.pose = None
        self.last_pose_at = None
        self.behavior_state = 'IDLE'
        self.selected = None
        self.connected = False
        self.future = None
        self.worker = ThreadPoolExecutor(max_workers=1,thread_name_prefix='gazebo_markers')
        self.create_subscription(TFMessage,'/_internal/sim_ground_truth_pose',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(String,'/behavior_status',self.on_behavior,1)
        self.status_pub = self.create_publisher(String,'/gazebo_visualization_status',1)
        self.create_timer(1/max(1.,min(20.,float(self.get_parameter('update_hz').value))),self.draw)
        self.get_logger().info('Gazebo trail and 3D bounding box: waiting for model world pose')

    def on_pose(self,msg):
        # Link entries are local transforms; only the model entry is world pose.
        root = next((t for t in msg.transforms if t.child_frame_id==self.robot_name),None)
        if root is None:
            return
        p,q = root.transform.translation,root.transform.rotation
        pose = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in pose):
            return
        self.pose = pose
        self.last_pose_at = time.monotonic()
        point = (p.x,p.y,.018)
        distance = math.dist(point,self.points[-1]) if self.points else 0.
        if distance>1.0:
            self.points.clear()  # Do not draw a fictitious teleport segment.
        if not self.points or distance>=self.spacing:
            self.points.append(point)

    def on_behavior(self,msg):
        try:
            data = json.loads(msg.data)
            state = data['state']
            if state=='RUNNING' and self.behavior_state in {'SELECTED','IDLE'}:
                self.points.clear()  # New task: clean trail. Resume: retain it.
                if self.pose:
                    self.points.append((self.pose[0],self.pose[1],.018))
            self.behavior_state = state
            self.selected = data.get('selected')
        except (ValueError,KeyError,TypeError):
            pass

    def draw(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                connected = bool(self.future.result())
            except Exception as exc:
                self.get_logger().error(f'Gazebo marker rendering failed: {exc}')
                connected = False
            if connected and not self.connected:
                self.get_logger().info('Gazebo MarkerManager connected: trail and bounding box visible')
            self.connected = connected
        if self.pose is None:
            return
        age = time.monotonic()-self.last_pose_at
        shutdown_s = float(self.get_parameter('shutdown_on_pose_loss_s').value)
        if shutdown_s>0 and age>shutdown_s:
            raise SystemExit(0)
        if age>.5:
            return
        trail = marker(1,'LINE_STRIP' if len(self.points)>1 else 'POINTS',
                       self.points,(.02,.55,1.))
        bounds = marker(2,'LINE_LIST',self.edges,(.15,1.,.20),self.pose,lifetime=1)
        self.future = self.worker.submit(_ign_markers.send,trail+' '+bounds,60)
        self.status_pub.publish(String(data=json.dumps(dict(
            connected=self.connected,trail_points=len(self.points),world_pose=self.pose,
            state=self.behavior_state,selected=self.selected))))

    def close(self):
        self.worker.shutdown(wait=True)
        _ign_markers.send('marker { ns: "bai06_visualization" action: DELETE_ALL }',60)


def main():
    rclpy.init()
    node = GazeboVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
