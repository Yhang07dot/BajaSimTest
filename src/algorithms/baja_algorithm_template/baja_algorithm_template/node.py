"""Algorithm team's editable entry point. No ground-truth subscriptions."""
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray
from baja_sim_interfaces.msg import RoadBoundaryArray


class Perception(Node):
    def __init__(self):
        super().__init__('team_perception')
        self.objects = self.create_publisher(Detection3DArray, '/perception/objects_3d', 10)
        self.boundaries = self.create_publisher(RoadBoundaryArray, '/perception/road_boundaries', 10)
        self.create_subscription(PointCloud2, '/sensors/lidar/points', self.on_cloud,
                                 qos_profile_sensor_data)
        self.get_logger().info('Template active: replace process_cloud() with your detector; currently publishes empty results.')

    def process_cloud(self, cloud):
        # TODO: decode cloud, remove invalid points, transform lidar_link points
        # into base_link using TF at cloud.header.stamp, then run your algorithm.
        # Return populated arrays. Do not replace the source timestamp with now().
        header = deepcopy(cloud.header)
        header.frame_id = 'base_link'
        return Detection3DArray(header=header), RoadBoundaryArray(header=header)

    def on_cloud(self, cloud):
        objects, boundaries = self.process_cloud(cloud)
        self.objects.publish(objects)
        self.boundaries.publish(boundaries)


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
