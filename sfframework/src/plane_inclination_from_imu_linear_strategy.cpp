#include <pluginlib/class_list_macros.hpp>

#include "sfframework/sf_strategy.hpp"
#include "sfframework/cluster_types.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <functional>
#include <memory>

#if __has_include("tf2_sensor_msgs/tf2_sensor_msgs.hpp")
  #include <boost/optional.hpp>
  #include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#elif __has_include("imu_transformer/tf2_sensor_msgs.h")
  #include "imu_transformer/tf2_sensor_msgs.h"
#endif

namespace sfframework
{
class PlaneInclinationFromImuLinear : public SFStrategy
{
  void onInitialize(grid_map::GridMap &grid_map) override
  {
    // subscribe to imu topic
    imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/camera/camera/accel/sample",
      10,
      std::bind(&PlaneInclinationFromImuLinear::imu_callback, this, std::placeholders::_1)
    );

    node_->declare_parameter(name_ + ".output_layer", "plane_inclination_imu");
    grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    latest_plane_model_.coefficients = Eigen::Vector4d(0, 0, 1, 0); // initialize to horizontal plane

    // time constant for low pass filter filter to smooth plane model updates from imu
    node_->declare_parameter(name_ + ".T", 0.5);
  }

  void onProcess(PartitioningContext & /*context*/, grid_map::GridMap &grid_map) override
  {
    auto output_layer = node_->get_parameter(name_ + ".output_layer").as_string();

    auto a = latest_plane_model_.coefficients(0);
    auto b = latest_plane_model_.coefficients(1);
    auto c = latest_plane_model_.coefficients(2);
    auto d = latest_plane_model_.coefficients(3);

    // Compute inclination field
    for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      grid_map::Position cell_position(0, 0);
      grid_map.getPosition(*iterator, cell_position);
      double x = cell_position(0);
      double y = cell_position(1);
      // Compute z from plane equation: ax + by + cz + d = 0  =>  z = -(a*x + b*y + d) / c

      double z = (- (a * x + b * y + d) / c);
      grid_map.at(output_layer, *iterator) = z;
    }
  }

private:
  // imu subscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  // latest plane model
  PlaneModel latest_plane_model_;

  // last imu message
  std::shared_ptr<sensor_msgs::msg::Imu> last_imu_message_;

  // callback to update plane model from imu
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!last_imu_message_) {
      last_imu_message_ = msg;
      return;
    }

    // time sample = time of current msg - time of last msg
    auto time_sample = rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_message_->header.stamp);
    auto Ts = time_sample.seconds();
    auto Ts_over_T = Ts / node_->get_parameter(name_ + ".T").as_double();

    auto gridmap_frame_id = node_->get_parameter("gridmap_frame_id").as_string();

    geometry_msgs::msg::TransformStamped imu_transform;

    // try to get the transform from the IMU frame to the grid map frame using tf2
    try {
      imu_transform = tf_buffer_->lookupTransform(
        gridmap_frame_id,
        msg->header.frame_id,
        // get time transform relative to the time of the IMU message
        msg->header.stamp
      );
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "%s: %s", name_.c_str(), ex.what());
      return;
    }

    sensor_msgs::msg::Imu transformed_msg;

    tf2::doTransform(*msg, transformed_msg, imu_transform);

    latest_plane_model_.coefficients = (1 - Ts_over_T) * latest_plane_model_.coefficients + Ts_over_T * Eigen::Vector4d(
      transformed_msg.linear_acceleration.x,
      transformed_msg.linear_acceleration.y,
      - transformed_msg.linear_acceleration.z,
      0
    );

    // RCLCPP_INFO(this->node_->get_logger(), "Plane model: a=%f, b=%f, c=%f, d=%f", latest_plane_model_.coefficients(0), latest_plane_model_.coefficients(1), latest_plane_model_.coefficients(2), latest_plane_model_.coefficients(3));

    // update last imu message
    last_imu_message_ = msg;
  }
};
} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::PlaneInclinationFromImuLinear, sfframework::SFStrategy)
