#include "sfframework/sfgenerator.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <rclcpp/executors.hpp>
#include <cmath>
#include <open3d/Open3D.h>
#include <Eigen/Dense>


SFGenerator::SFGenerator()
  : Node("sfgenerator"),
    filter_loader_("sfframework", "sfframework::FilterBase")
{
  this->lidar_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = lidar_callback_group_;

  // Add parameters a and b with type float and default values
  this->declare_parameter("A", 0.001);
  this->declare_parameter("sigma", 0.5);

  auto declared_classes = filter_loader_.getDeclaredClasses();
  RCLCPP_INFO(this->get_logger(), "[%s] Querying available filter plugins:", this->get_name());
  for (const auto & cls : declared_classes) {
    RCLCPP_INFO(this->get_logger(), "  %s", cls.c_str());
  }

  this->declare_parameter("filter_plugins", std::vector<std::string>());
  auto filter_names = this->get_parameter("filter_plugins").as_string_array();

  for (const auto & name : filter_names) {
    this->declare_parameter(name + ".plugin", "");
    std::string type = this->get_parameter(name + ".plugin").as_string();
    try {
      auto filter = filter_loader_.createSharedInstance(type);
      filter->initialize(name, this);
      filters_.push_back(filter);
      RCLCPP_INFO(this->get_logger(), "Successfully loaded filter plugin '%s' of type '%s'", name.c_str(), type.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for filter '%s'. Error: %s", name.c_str(), ex.what());
    }
  }

  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  // qos_profile.best_effort();

  pointcloud2_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot/top_laser/points",
    qos_profile,
    std::bind(&SFGenerator::pointcloud2_topic_callback, this, std::placeholders::_1),
    sub_opt);
  grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
  intermediate_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("intermediate_pointcloud", 10);

  grid_map_.setFrameId("robot_base_footprint");
  grid_map_.setGeometry(grid_map::Length(5.0, 5.0), 0.10);
  grid_map_.add("potential", 0);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  RCLCPP_INFO(this->get_logger(),
    "Node [%s] started.", this->get_name());
}

void SFGenerator::rosToOpen3d(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_pc, const std::shared_ptr<open3d::geometry::PointCloud>& o3d_pc, float dist_limit){
    // Safety checks
    if (ros_pc->height == 0 || ros_pc->width == 0) {
        return;
    }

    // Pre-allocate memory to speed up the vector push_back
    size_t num_points = ros_pc->height * ros_pc->width;
    o3d_pc->points_.reserve(num_points);

    // Create ROS 2 Iterators for x, y, z
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*ros_pc, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*ros_pc, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*ros_pc, "z");

    // Iterate and Convert
    float dist_limit_sqrd = dist_limit * dist_limit;
    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        // filter out invalid points
        if (Eigen::Vector2d(*iter_x, *iter_y).squaredNorm() <= dist_limit_sqrd && std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
            o3d_pc->points_.emplace_back(*iter_x, *iter_y, *iter_z);
        }
    }
}

void SFGenerator::Open3dToRos(const std::shared_ptr<open3d::geometry::PointCloud>& o3d_pc, const sensor_msgs::msg::PointCloud2::SharedPtr& ros_pc, std::string frame_id)
{
  // Set the header
  ros_pc->header.stamp = this->get_clock()->now();
  ros_pc->header.frame_id = frame_id;

  // Prepare the modifier 
  sensor_msgs::PointCloud2Modifier modifier(*ros_pc);
  
  // Define ONLY x, y, z fields
  modifier.setPointCloud2FieldsByString(1, "xyz");

  // 3. Resize the ROS message to match the Open3D cloud size
  modifier.resize(o3d_pc->points_.size());

  // 4. Create Iterators
  sensor_msgs::PointCloud2Iterator<float> iter_x(*ros_pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*ros_pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*ros_pc, "z");

  // 5. Loop and Fill
  for (const auto& point : o3d_pc->points_) {
    *iter_x = point(0); // x
    *iter_y = point(1); // y
    *iter_z = point(2); // z

    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
}

void SFGenerator::pointcloud2_topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  auto intermediate_pc = std::make_shared<sensor_msgs::msg::PointCloud2>();
  auto A = this->get_parameter("A").as_double();
  auto two_sigma_sqrd = this->get_parameter("sigma").as_double();
  two_sigma_sqrd = two_sigma_sqrd * two_sigma_sqrd;
  two_sigma_sqrd = two_sigma_sqrd * 2.0;
  
  auto distance_threshold = this->get_parameter("sigma").as_double() * 3.0;
  
  
  auto tstart = this->get_clock()->now();
  
  this->cloud = *msg;
  
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
      
      // float max_range = 2 * distance_threshold; // meters
      float max_range = 10.0; 
      
      auto o3d_pc = std::make_shared<open3d::geometry::PointCloud>();
      rosToOpen3d(std::make_shared<sensor_msgs::msg::PointCloud2>(cloud), o3d_pc, max_range);
      
      for (const auto & filter : filters_) {
        o3d_pc = filter->filter(o3d_pc);
      }
      
      Open3dToRos(o3d_pc, intermediate_pc, grid_map_.getFrameId());
    
      // Publish intermediate point cloud
      this->intermediate_pointcloud_publisher_->publish(*intermediate_pc);
      
      // Reset potential field
      grid_map_.get("potential").setZero();
      
      // Calculate potential field from o3d point cloud
      for (const auto& point : o3d_pc->points_)
      {
        // grid_map_.atPosition("potential", ...) = 1.0;
        auto point_position_on_grid = grid_map::Position(point(0), point(1));
        
        // grid_map_.atPosition("potential", position) = 1.0;
        
        // iterate over every every gridmap cell
        for(grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
          grid_map::Position cell_position(0, 0);
          grid_map_.getPosition(*iterator, cell_position);
          double distance_sqrd = (point_position_on_grid - cell_position).squaredNorm();
          
          if (distance_sqrd < distance_threshold ){
            // Potential field decreases with distance_sqrd (e.g., Gaussian)
            double potential = A * std::exp(-distance_sqrd /  two_sigma_sqrd);
            if(grid_map_.at("potential", *iterator) < potential){
              grid_map_.at("potential", *iterator) = potential;
            }
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
      // RCLCPP_INFO(this->get_logger(), "Cycle time: %f ms", (tend - tstart).nanoseconds() / 1000000.0);
      
    }
    
    
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SFGenerator>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  
  executor.add_node(node);
  executor.spin();
  // rclcpp::spin(std::make_shared<SFGenerator>());
  // rclcpp::spin(std::make_shared<SFGenerator>(), executor);
  rclcpp::shutdown();
  return 0;
}