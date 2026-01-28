#include "sfframework/sfgenerator.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/executors.hpp>
#include <cmath>
#include <open3d/Open3D.h>
#include <Eigen/Dense>


SFGenerator::SFGenerator()
  : Node("sfgenerator"),
    filter_loader_("sfframework", "sfframework::FilterBase"),
    partitioner_loader_("sfframework", "sfframework::PartitioningStrategy")
{
  this->lidar_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = lidar_callback_group_;

  // Add parameters a and b with type float and default values
  this->declare_parameter("A", 0.001);
  this->declare_parameter("sigma", 0.5);

  this->declare_parameter("publish_filtered_pointcloud", true);
  
  this->declare_parameter("gridmap_frame_id", "robot_odom");
  this->declare_parameter("gridmap_center_target_frame_id", "robot_base_footprint");

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
  
  auto partitioner_declared_classes = partitioner_loader_.getDeclaredClasses();
  RCLCPP_INFO(this->get_logger(), "[%s] Querying available partitioner plugins:", this->get_name());
  for (const auto & cls : partitioner_declared_classes) {
    RCLCPP_INFO(this->get_logger(), "  %s", cls.c_str());
  }

  this->declare_parameter("partitioner_plugins", std::vector<std::string>());
  auto partitioner_names = this->get_parameter("partitioner_plugins").as_string_array();

  for (const auto & name : partitioner_names) {
    this->declare_parameter(name + ".plugin", "");
    std::string type = this->get_parameter(name + ".plugin").as_string();
    try {
      auto partitioner = partitioner_loader_.createSharedInstance(type);
      partitioner->initialize(this, name);
      partitioners_.push_back(partitioner);
      RCLCPP_INFO(this->get_logger(), "Successfully loaded partitioner plugin '%s' of type '%s'", name.c_str(), type.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for partitioner '%s'. Error: %s", name.c_str(), ex.what());
    }
  }

  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  // qos_profile.best_effort();

  pointcloud2_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/points",
    qos_profile,
    std::bind(&SFGenerator::pointcloud2_topic_callback, this, std::placeholders::_1),
    sub_opt);
  grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
  filtered_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);

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

  modifier.resize(o3d_pc->points_.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*ros_pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*ros_pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*ros_pc, "z");

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
  auto A = this->get_parameter("A").as_double();
  auto two_sigma_sqrd = this->get_parameter("sigma").as_double();
  two_sigma_sqrd = two_sigma_sqrd * two_sigma_sqrd;
  two_sigma_sqrd = two_sigma_sqrd * 2.0;
  
  auto distance_threshold = this->get_parameter("sigma").as_double() * 3.0;
  
  
  auto tstart = this->get_clock()->now();
  
  this->cloud = *msg;
  
  grid_map_.setFrameId(this->get_parameter("gridmap_frame_id").as_string());

  // Set gridmap position 
  geometry_msgs::msg::TransformStamped tf_grid_frame_to_grid_target;
  try{
    tf_grid_frame_to_grid_target = tf_buffer_.get()->lookupTransform(
      this->get_parameter("gridmap_frame_id").as_string(),  // source frame
    this->get_parameter("gridmap_center_target_frame_id").as_string(),  // target frame
    tf2::TimePointZero);  // get the latest available
  }
  catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }
  grid_map_.setPosition(grid_map::Position(tf_grid_frame_to_grid_target.transform.translation.x, tf_grid_frame_to_grid_target.transform.translation.y));

  // Transform point cloud to girdmap frame
  geometry_msgs::msg::TransformStamped tf_cloud_to_grid_frame;
  try{
    tf_cloud_to_grid_frame = tf_buffer_.get()->lookupTransform(
    this->get_parameter("gridmap_frame_id").as_string(),  // target frame
    cloud.header.frame_id,  // source frame
    tf2::TimePointZero);  // get the latest available
  }
  catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }
  tf2::doTransform(cloud, cloud, tf_cloud_to_grid_frame);
  // float max_range = 2 * distance_threshold; // meters
  float max_range = 10.0; 
  
  auto o3d_pc = std::make_shared<open3d::geometry::PointCloud>();
  rosToOpen3d(std::make_shared<sensor_msgs::msg::PointCloud2>(cloud), o3d_pc, max_range);

  // run every filter from list of plugins in order
  for (const auto & filter : filters_) {
    o3d_pc = filter->filter(o3d_pc);
  }
  
  // publish filtered point cloud if parameter is set
  if (this->get_parameter("publish_filtered_pointcloud").as_bool()) {
    auto intermediate_pc = std::make_shared<sensor_msgs::msg::PointCloud2>();

    Open3dToRos(o3d_pc, intermediate_pc, this->get_parameter("gridmap_frame_id").as_string());
    this->filtered_pointcloud_publisher_->publish(*intermediate_pc);
  }
  
  // run every partitioner from list of plugins in order
  PartitioningContext context;
  context.cloud = o3d_pc;
  for (const auto & partitioner : partitioners_) {
    partitioner->process(context);
  }

  o3d_pc = o3d_pc->SelectByIndex(context.clusters_registry["ground"][0].indices, true);

  // Reset potential field
  grid_map_.get("potential").setZero();
  
  // Calculate potential field from o3d point cloud
  for (const auto& point : o3d_pc->points_)
  {
    auto point_position_on_grid = grid_map::Position(point(0), point(1));
    
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
  
  //sleep for 3 seconds for debugging high latency
  // std::this_thread::sleep_for(std::chrono::seconds(3));
  

  // Set the timestamp and publish the grid map
  grid_map::Time timestamp(this->get_clock()->now().nanoseconds());
  grid_map_.setTimestamp(timestamp);
  auto message = grid_map::GridMapRosConverter::toMessage(grid_map_);
  // print gridmap position for debugging
  
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
  rclcpp::shutdown();
  return 0;
}