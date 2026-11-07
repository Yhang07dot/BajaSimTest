"""Fixed-route control, timestamped ground truth and visualization."""
import copy
import json
import math
from pathlib import Path
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray
from baja_sim_interfaces.msg import RoadBoundary, RoadBoundaryArray, SurfaceRegion, SurfaceRegionArray

from .geometry import PoseHistory, points_in_box, to_local, pursuit_yaw_rate
from .ros_utils import (seconds, pose_arrays, cloud_xyz, detection_array,
                        detection_markers, transform, boundary_markers)


class Platform(Node):
    def __init__(self):
        super().__init__('simulation_platform')
        self.declare_parameter('scene_file','')
        self.scene=json.loads(Path(self.get_parameter('scene_file').value).read_text())
        self.cfg=self.scene['config']
        self.path=np.array(self.scene['path'])
        self.tree=cKDTree(self.path[:,:2])
        self.history=PoseHistory()
        self.odom=None;self.queue=deque(maxlen=10);self.last_cloud=-1.
        self.cmd=self.create_publisher(Twist,'/model/test_vehicle/cmd_vel',10)
        self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self)
        stamp=self.get_clock().now().to_msg()
        self.static_tf.sendTransform([
            transform('base_link','lidar_link',self.scene['lidar_mount'],[0,0,0,1],stamp),
            transform('base_link','camera_link',self.scene['camera_mount'],[0,0,0,1],stamp),
            transform('camera_link','camera_optical_frame',[0,0,0],Rotation.from_euler('xyz',[-math.pi/2,0,-math.pi/2]).as_quat(),stamp)])
        self.truth=self.create_publisher(Detection3DArray,'/ground_truth/eligible_objects',10)
        self.roi=self.create_publisher(Detection3DArray,'/ground_truth/roi_objects',10)
        self.boundary_pub=self.create_publisher(RoadBoundaryArray,'/ground_truth/road_boundaries',10)
        self.surface_pub=self.create_publisher(SurfaceRegionArray,'/ground_truth/surface_regions',10)
        self.truth_markers=self.create_publisher(MarkerArray,'/ground_truth/markers',10)
        self.boundary_markers=self.create_publisher(MarkerArray,'/ground_truth/boundary_markers',10)
        self.pred_markers=self.create_publisher(MarkerArray,'/perception/markers',10)
        self.pred_boundaries=self.create_publisher(MarkerArray,'/perception/boundary_markers',10)
        self.status=self.create_publisher(String,'/simulation/status',10)
        self.create_subscription(Odometry,'/ground_truth/odom',self.on_odom,10)
        self.create_subscription(PointCloud2,'/sensors/lidar/points',self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(Detection3DArray,'/perception/objects_3d',lambda m:self.pred_markers.publish(detection_markers(m,'predictions',(1.,0.6,0.,0.35))),10)
        self.create_subscription(RoadBoundaryArray,'/perception/road_boundaries',lambda m:self.pred_boundaries.publish(boundary_markers(m)),10)
        self.create_timer(0.05,self.control)
        self.create_timer(1.0,self.show_world)
        self.get_logger().info(f'{self.scene["length"]:.1f}m loop; {len(self.scene["objects"])} safe objects; seed={self.cfg["seed"]}')

    def on_odom(self,msg):
        self.odom=msg
        p,q=pose_arrays(msg.pose.pose)
        self.history.add(seconds(msg.header.stamp),p,q)
        self.tf.sendTransform(transform('map','base_link',p,q,msg.header.stamp))
        self.drain()

    def control(self):
        if self.odom is None:return
        p,q=pose_arrays(self.odom.pose.pose)
        error,i=self.tree.query(p[:2])
        target=(i+round(self.cfg['vehicle']['lookahead']/self.cfg['track']['sample_step'])) % len(self.path)
        delta=to_local([[*self.path[target,:2],p[2]]],p,q)[0]
        cmd=Twist()
        # Stop on localization loss or excessive deviation instead of leaving the safe corridor.
        age=seconds(self.get_clock().now().to_msg())-seconds(self.odom.header.stamp)
        if error < 0.65 and age < 0.5:
            cmd.linear.x=self.cfg['vehicle']['speed']
            cmd.angular.z=pursuit_yaw_rate(cmd.linear.x,delta)
        self.cmd.publish(cmd)

    def on_cloud(self,msg):
        t=seconds(msg.header.stamp)
        if t<self.last_cloud:self.queue.clear()
        self.last_cloud=t
        self.queue.append(msg)
        self.drain()

    def drain(self):
        while self.queue and self.history.items:
            msg=self.queue[0];t=seconds(msg.header.stamp)
            if t>self.history.items[-1][0]:return
            self.queue.popleft()
            pose=self.history.at(t)
            if pose is None:continue
            self.publish_frame(msg,pose)

    def publish_frame(self,cloud,pose):
        header=copy.deepcopy(cloud.header);header.frame_id='base_link'
        points=cloud_xyz(cloud)+self.scene['lidar_mount']
        points=points @ Rotation.from_quat(pose[1]).as_matrix().T + pose[0]
        e=self.cfg['evaluation']
        eligible=[];roi=[]
        centres=to_local([o['position'] for o in self.scene['objects']],*pose)
        for obj,p in zip(self.scene['objects'],centres):
            if p[0]<e['min_forward'] or np.linalg.norm(p[:2])>e['max_distance'] or abs(math.atan2(p[1],p[0]))>math.radians(e['half_fov_deg']):continue
            roi.append(obj)
            # Actual returns account for beam gaps, geometric blind region and occlusion.
            if np.count_nonzero(points_in_box(points,obj))>=e['min_lidar_hits']:
                eligible.append(obj)
        self.roi.publish(detection_array(roi,header,pose))
        self.truth.publish(detection_array(eligible,header,pose))
        boundaries=RoadBoundaryArray(header=header)
        for side,key in [(1,'left'),(2,'right')]:
            local=to_local(self.scene['boundaries'][key],*pose)
            valid=(local[:,0]>=e['min_forward']) & (np.linalg.norm(local[:,:2],axis=1)<=e['max_distance'])
            # Keep curve order from nearest forward samples; split at wrap or visibility gaps.
            indices=np.flatnonzero(valid)
            for group in np.split(indices,np.flatnonzero(np.diff(indices)>1)+1):
                if len(group)<2:continue
                b=RoadBoundary(side=side,confidence=1.)
                b.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in local[group]]
                boundaries.boundaries.append(b)
        self.boundary_pub.publish(boundaries)
        self.boundary_markers.publish(boundary_markers(boundaries))
        surfaces=SurfaceRegionArray(header=header)
        for region in self.scene['regions']:
            p=to_local(region['polygon'],*pose)
            if np.linalg.norm(np.mean(p[:,:2],axis=0))>40:continue
            r=SurfaceRegion(class_id=region['class_id'],confidence=1.,height=region['height'])
            r.polygon=[Point(x=float(a[0]),y=float(a[1]),z=float(a[2])) for a in p]
            surfaces.regions.append(r)
        self.surface_pub.publish(surfaces)

    def show_world(self):
        header=Header(frame_id='map',stamp=self.get_clock().now().to_msg())
        self.truth_markers.publish(detection_markers(detection_array(self.scene['objects'],header),'truth',(0.,1.,0.,0.18)))
        state={'seed':self.cfg['seed'],'objects':len(self.scene['objects']),'odometry_received':self.odom is not None,'cloud_time':self.last_cloud}
        if self.odom:
            p,_=pose_arrays(self.odom.pose.pose)
            state['cross_track_error_m']=float(self.tree.query(p[:2])[0])
        self.status.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args);node=Platform()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.cmd.publish(Twist())
        node.destroy_node();rclpy.try_shutdown()
