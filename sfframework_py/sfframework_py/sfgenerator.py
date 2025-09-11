import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs_py.point_cloud2 as pc2
from sfframework_msgs.msg import ScalarField2D
import tf2_sensor_msgs.tf2_sensor_msgs as tf2s
import tf2_ros


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

        # TODO: unhardcode published topic parameters
        self.current_field = ScalarField2D()

        self.current_field.info.width = 25
        self.current_field.info.height = 25
        self.current_field.info.resolution = 0.5

        self.current_field.info.origin.position.x = - (self.current_field.info.width * self.current_field.info.resolution) / 2.0
        self.current_field.info.origin.position.y = - (self.current_field.info.height * self.current_field.info.resolution) / 2.0

        self.current_field.data = np.zeros((self.current_field.info.height, self.current_field.info.width), dtype=np.float32).flatten().tolist()

        self.current_field.header.frame_id = "robot_base_footprint"

        self.field_publisher = self.create_publisher(
            ScalarField2D,
            '/sfframework/scalar_field',
            10)
        
        # Initialize TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        

    def laser_msg_callback(self, msg: LaserScan):
        pc2_msg = lg.LaserProjection().projectLaser(msg)
        #publish pointcloud2
        self.pc2_publisher.publish(pc2_msg)

    def pc2_msg_callback(self, msg: PointCloud2):
        # convert to scalar field
        # clear current field
        self.current_field.data = np.zeros((self.current_field.info.height, self.current_field.info.width), dtype=np.float32).flatten().tolist()

        # transform pointcloud to scalar field frame_id
        field_frame_id = self.current_field.header.frame_id
        pc2_frame_id = msg.header.frame_id

        try:
            # Lookup transform from pointcloud frame to field frame
            current_transform = self.tf_buffer.lookup_transform(
            target_frame=field_frame_id,
            source_frame=pc2_frame_id,
            time=rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            return

        transformed_pc2 = tf2s.do_transform_cloud(msg, current_transform)

        

        points = pc2.read_points_numpy(transformed_pc2, field_names=("x", "y", "z"), skip_nans=True)

        for point in points:
            x, y, z = point
                
            # get (closest) x and y cell
            x_idx = int(
                (x - self.current_field.info.origin.position.x) / self.current_field.info.resolution
                )
            y_idx = int(
                (y - self.current_field.info.origin.position.y) / self.current_field.info.resolution
                )
            
            if x_idx < 0 or x_idx >= self.current_field.info.width or y_idx < 0 or y_idx >= self.current_field.info.height:
                continue
            
            # update cell value 
            self.current_field.data[y_idx * self.current_field.info.width + x_idx] = 1.0

        self.current_field.header.stamp = self.get_clock().now().to_msg()
        self.field_publisher.publish(self.current_field)



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