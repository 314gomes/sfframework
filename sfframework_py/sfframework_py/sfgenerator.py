import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # TODO: unhardcode topic names (use parameters and probably a list...)
        # TODO: it makes sense to put this in another node... maybe one that accepts multiple lasers and publishes to a single pointcloud2 topic
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/robot/front_laser/scan',
            self.laser_msg_callback,
            10)
        # create pointcloud2 publisher
        self.pc2_publisher = self.create_publisher(
            PointCloud2,
            '/sfframework/pointcloud2',
            10)
        
        # create pointcloud2 subscriber
        self.pc2_subscription = self.create_subscription(
            PointCloud2,
            '/sfframework/pointcloud2',
            self.pc2_msg_callback,
            10)

        self.laser_subscription  # prevent unused variable warning

    def laser_msg_callback(self, msg: LaserScan):
        pc2_msg = lg.LaserProjection().projectLaser(msg)
        #publish pointcloud2
        self.pc2_publisher.publish(pc2_msg)

    def pc2_msg_callback(self, msg: PointCloud2):
        # convert to scalar field
        msg.data
        
        points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        for point in points:
            x, y, z = point
            

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()