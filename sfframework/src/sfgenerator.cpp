#include "sfframework/sfgenerator.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>


SFGenerator::SFGenerator()
  : Node("sfgenerator")
{
  // Add parameters a and b with type float and default values
  this->declare_parameter("A", 0.1);
  this->declare_parameter("sigma", 0.5);


  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/robot/front_laser/scan", 10,
    std::bind(&SFGenerator::laser_topic_callback, this, std::placeholders::_1));

  grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);

  grid_map_.setFrameId("robot_base_footprint");
  grid_map_.setGeometry(grid_map::Length(5.0, 5.0), 0.10);
  grid_map_.add("potential", 0);

  RCLCPP_INFO(this->get_logger(),
    "Hello World! This is grid_map example. Resulting map will have %d cells.",
    grid_map_.getSize().prod());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}


void SFGenerator::laser_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  auto A = this->get_parameter("A").as_double();
  auto two_sigma_sqrd = this->get_parameter("sigma").as_double();
  two_sigma_sqrd = two_sigma_sqrd * two_sigma_sqrd;
  two_sigma_sqrd = two_sigma_sqrd * 2.0;

  auto distance_threshold = this->get_parameter("sigma").as_double() * 3.0;


  auto tstart = this->get_clock()->now();

  this->projector_.projectLaser(*msg, cloud);

  // iterate over every every gridmap cell
  for(grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    grid_map_.at("potential", *iterator) = grid_map_.at("potential", *iterator) * 0.95;
  }

  // Transform point cloud to target frame
  geometry_msgs::msg::TransformStamped transform_stamped;
  try{
    // transform_stamped = tf_buffer_.lookupTransform(
    transform_stamped = tf_buffer_.get()->lookupTransform(
      grid_map_.getFrameId(),  // target frame
      cloud.header.frame_id,  // source frame
      tf2::TimePointZero);  // get the latest available
  }
  catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  tf2::doTransform(cloud, cloud, transform_stamped);

  // Reset potential field
  grid_map_.get("potential").setZero();

  // Calculate potential field from point cloud
  for (sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x"), iter_y(cloud, "y");
       iter_x != iter_x.end(); ++iter_x, ++iter_y)
  {
    // grid_map_.atPosition("potential", (*iter_x, *iter_y) = 1.0;
    auto position = grid_map::Position(*iter_x, *iter_y);

    // grid_map_.atPosition("potential", position) = 1.0;
    
    // iterate over every every gridmap cell
    for(grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position cell_position(0, 0);
      grid_map_.getPosition(*iterator, cell_position);
      double distance_sqrd = (position - cell_position).squaredNorm();
      
      if (distance_sqrd < distance_threshold ){
        // Potential field decreases with distance_sqrd (e.g., Gaussian)
        double potential = A * std::exp(-distance_sqrd /  two_sigma_sqrd);
        
        grid_map_.at("potential", *iterator) += potential;
      }

    }


  }


  
  // Set the timestamp and publish the grid map
  // grid_map_.setTimestamp(this->now().nanoseconds());
  grid_map::Time timestamp(this->get_clock()->now().nanoseconds());
  grid_map_.setTimestamp(timestamp);
  auto message = grid_map::GridMapRosConverter::toMessage(grid_map_);
  grid_map_publisher_->publish(*message);

  auto tend = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Cycle time: %f ms", (tend - tstart).nanoseconds() / 1000000.0);

}


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SFGenerator>());
  rclcpp::shutdown();
  return 0;
}