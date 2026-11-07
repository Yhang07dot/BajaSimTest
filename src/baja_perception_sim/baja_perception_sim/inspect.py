"""Small live sensor checker, no recording required."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import Odometry
from .ros_utils import cloud_xyz,seconds


class Inspector(Node):
    def __init__(self):
        super().__init__('sensor_inspector')
        self.counts={'cloud':0,'image':0,'camera_info':0,'odom':0};self.details={}
        self.first=None;self.last=None;self.odom_start=None
        self.create_subscription(PointCloud2,'/sensors/lidar/points',self.cloud,qos_profile_sensor_data)
        self.create_subscription(Image,'/sensors/front_camera/image_raw',self.image,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/sensors/front_camera/camera_info',lambda m:self.info(m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/ground_truth/odom',self.odom,10)
        self.started=time.monotonic()

    def cloud(self,m):
        self.counts['cloud']+=1
        t=seconds(m.header.stamp)
        if self.first is None:self.first=t
        self.last=t
        p=cloud_xyz(m)
        angles=np.unique(np.round(np.degrees(np.arctan2(p[:,2],np.hypot(p[:,0],p[:,1]))),1))
        self.details['cloud']={'width':m.width,'height':m.height,'finite_points':len(p),'frame':m.header.frame_id,
                               'fields':[f.name for f in m.fields],'elevation_min_max_deg':([float(angles.min()),float(angles.max())] if len(angles) else None)}

    def image(self,m):
        self.counts['image']+=1;self.details['image']={'width':m.width,'height':m.height,'encoding':m.encoding,'frame':m.header.frame_id}

    def info(self,m):
        self.counts['camera_info']+=1;self.details['camera_info']={'fx':m.k[0],'frame':m.header.frame_id}

    def odom(self,m):
        self.counts['odom']+=1
        p=m.pose.pose.position
        if self.odom_start is None:self.odom_start=[p.x,p.y,p.z]
        self.details['odom']={'position':[p.x,p.y,p.z],'start':self.odom_start}


def main(args=None):
    rclpy.init(args=args);node=Inspector()
    try:
        while time.monotonic()-node.started<20:rclpy.spin_once(node,timeout_sec=0.2)
        out={'counts':node.counts,**node.details}
        if node.first is not None and node.last>node.first:out['cloud_sim_hz']=(node.counts['cloud']-1)/(node.last-node.first)
        print(json.dumps(out,indent=2))
        ok=all(node.counts.values()) and node.details['cloud']['finite_points']>0 and node.details['cloud']['height']==16 and node.details['cloud']['width']==2000
    finally:node.destroy_node();rclpy.try_shutdown()
    if not ok:raise SystemExit(1)
