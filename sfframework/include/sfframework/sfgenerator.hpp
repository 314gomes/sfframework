#include "rclcpp/rclcpp.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


class SFGenerator : public rclcpp::Node{
	public:
		SFGenerator();

	private:
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
		rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
		void laser_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
		
		grid_map::GridMap grid_map_;

		laser_geometry::LaserProjection projector_;
  		sensor_msgs::msg::PointCloud2 cloud;

		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	};