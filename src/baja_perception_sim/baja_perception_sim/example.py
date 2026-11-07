"""Minimal algorithm integration example: empty results, no ground-truth access."""
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray
from baja_sim_interfaces.msg import RoadBoundaryArray


class Example(Node):
    def __init__(self):
        super().__init__('example_algorithm')
        self.pub=self.create_publisher(Detection3DArray,'/perception/objects_3d',10)
        self.boundaries=self.create_publisher(RoadBoundaryArray,'/perception/road_boundaries',10)
        self.create_subscription(PointCloud2,'/sensors/lidar/points',self.receive,qos_profile_sensor_data)
        self.get_logger().info('Interface example only: publishes empty detections and boundaries. Replace receive() with your algorithm.')

    def receive(self,cloud):
        header=copy.deepcopy(cloud.header)
        # If producing boxes, transform points from lidar_link to base_link at
        # cloud.header.stamp before computing / publishing their coordinates.
        header.frame_id='base_link'
        self.pub.publish(Detection3DArray(header=header))
        self.boundaries.publish(RoadBoundaryArray(header=header))


def main(args=None):
    rclpy.init(args=args);node=Example()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
