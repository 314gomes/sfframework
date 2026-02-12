#include <pluginlib/class_list_macros.hpp>
#include "sfframework/sf_strategy.hpp"
#include "sfframework/sfgenerator_utils.hpp"
#include "sfframework/cluster_types.hpp"
#include <omp.h>
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Geometry>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "imu_transformer/tf2_sensor_msgs.h"

namespace sfframework
{
  class GaussianRepulsion : public SFStrategy
  {
    void onInitialize(grid_map::GridMap &grid_map) override
    {
      node_->declare_parameter(name_ + ".A", 1.0);
      node_->declare_parameter(name_ + ".sigma", 0.5);
      node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
      node_->declare_parameter(name_ + ".invert_selection", false);
      node_->declare_parameter(name_ + ".output_layer", "gaussian_repulsion");
      grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    }

    void onProcess(PartitioningContext &context, grid_map::GridMap &grid_map) override
    {
      std::shared_ptr<const open3d::geometry::PointCloud> o3d_pc;

      // attempt to get point cloud from context
      try
      {
        o3d_pc = sfframework::SFGeneratorUtils::cloud_from_tags(
          context,
          node_->get_parameter(name_ + ".input_tags").as_string_array(),
          node_->get_parameter(name_ + ".invert_selection").as_bool()
        );
      }
      catch (const std::out_of_range &e)
      {
        RCLCPP_WARN(node_->get_logger(), "SF strategy plugin [%s] tried to find a list of clusters at a tag but it is missing. Skipping...", name_.c_str());
        return;
      }

      // Reset the layer to zero before accumulating new values
      grid_map[this->node_->get_parameter(name_ + ".output_layer").as_string()].setConstant(0.0);

      // Distance above which the potential is negligible and we can skip computation for efficiency
      auto sigma = this->node_->get_parameter(name_ + ".sigma").as_double();
      auto distance_threshold_sqrd = sigma * sigma * 9.0;
      auto A = this->node_->get_parameter(name_ + ".A").as_double();
      auto two_sigma_sqrd = 2.0 * sigma * sigma;
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();

      // Compute potential field using omp
      #pragma omp parallel for
      for (size_t i = 0; i < o3d_pc->points_.size(); ++i)
      {
        auto point = o3d_pc->points_[i];
        auto point_position_on_grid = grid_map::Position(point(0), point(1));

        // iterate over every every gridmap cell
        for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
        {
          grid_map::Position cell_position(0, 0);
          grid_map.getPosition(*iterator, cell_position);
          double distance_sqrd = (point_position_on_grid - cell_position).squaredNorm();

          if (distance_sqrd < distance_threshold_sqrd)
          {
            // Potential field decreases with distance_sqrd
            double potential = A * std::exp(-distance_sqrd / two_sigma_sqrd);
            if (grid_map.at(output_layer, *iterator) < potential)
            {
              grid_map.at(output_layer, *iterator) = potential;
            }
          }
        }
      }
    }
  };

  // Resting attractive potential centered at a user-defined point
  class ParabolicRest : public SFStrategy
  {
    void onInitialize(grid_map::GridMap &grid_map) override
    {
      node_->declare_parameter(name_ + ".zeta", 0.1);
      node_->declare_parameter(name_ + ".center_x", 0.0);
      node_->declare_parameter(name_ + ".center_y", 0.0);
      node_->declare_parameter(name_ + ".output_layer", "parabolic_rest");
      grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    }

    void onProcess(PartitioningContext & /*context*/, grid_map::GridMap &grid_map) override
    {
      auto zeta = this->node_->get_parameter(name_ + ".zeta").as_double();
      auto center_x = this->node_->get_parameter(name_ + ".center_x").as_double();
      auto center_y = this->node_->get_parameter(name_ + ".center_y").as_double();
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();

      // Compute potential field using omp
      // #pragma omp parallel for
      for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
      {
        grid_map::Position cell_position(0, 0);
        grid_map.getPosition(*iterator, cell_position);
        double distance_sqrd = (cell_position(0) - center_x) * (cell_position(0) - center_x) +
                              (cell_position(1) - center_y) * (cell_position(1) - center_y);

        // Parabolic potential field increases with distance_sqrd
        double potential = zeta * distance_sqrd;
        grid_map.at(output_layer, *iterator) = potential;
      }
    }
  };

  class WeightedSummation : public SFStrategy
  {
    void onInitialize(grid_map::GridMap &grid_map) override
    {
      node_->declare_parameter(name_ + ".input_layers", std::vector<std::string>());
      node_->declare_parameter(name_ + ".weights", std::vector<double>());
      node_->declare_parameter(name_ + ".output_layer", "potential");

      grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    }

    void onProcess(PartitioningContext & /*context*/, grid_map::GridMap &grid_map) override
    {
      auto input_layers = this->node_->get_parameter(name_ + ".input_layers").as_string_array();
      auto weights = this->node_->get_parameter(name_ + ".weights").as_double_array();
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();

      if (input_layers.size() != weights.size())
      {
        RCLCPP_ERROR(node_->get_logger(), "SF strategy plugin [%s] has a different number of input layers and weights. Skipping...", name_.c_str());
        return;
      }

      // Check if every input layer exists
      for (size_t i = 0; i < input_layers.size(); ++i){
        if (!grid_map.exists(input_layers[i]))
        {
          RCLCPP_WARN(node_->get_logger(), "SF strategy plugin [%s] tried to find an input layer that does not exist. Skipping...", name_.c_str());
          return;
        }
      }
      
      // Compute weighted summation
      grid_map[output_layer].setConstant(0.0);
      for (size_t i = 0; i < input_layers.size(); ++i)
      {
        grid_map[output_layer] += weights[i] * grid_map[input_layers[i]];
      }

    }
  };

  class PlaneInclination : public SFStrategy
  {
    void onInitialize(grid_map::GridMap &grid_map) override
    {
      node_->declare_parameter(name_ + ".input_tag", "ground");

      node_->declare_parameter(name_ + ".output_layer", "plane_inclination");
      grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    }

    void onProcess(PartitioningContext & context, grid_map::GridMap &grid_map) override
    {
      auto input_tag = this->node_->get_parameter(name_ + ".input_tag").as_string();
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();
      // minimum value for c to avoid division by zero resulting in overly inclined plane
      double threshold = 0.01;

      PlaneModel plane;
      try
      {
        // plane = (PlaneModel) context->clusters_registry.at(input_tag).at(0).attributes.at("model");
        plane = std::any_cast<PlaneModel>(context.clusters_registry.at(input_tag).at(0).attributes.at("model"));
      }
      catch (const std::out_of_range &e)
      {
        RCLCPP_WARN(node_->get_logger(), "SF strategy plugin [%s] tried to find a plane model in the cluster attributes but it is missing. Skipping...", name_.c_str());
        return;
      }

      auto a = plane.coefficients(0);
      auto b = plane.coefficients(1);
      auto c = plane.coefficients(2);
      auto d = plane.coefficients(3);

      // print plane model for debugging
      // RCLCPP_INFO(node_->get_logger(), "Plane model: a=%f, b=%f, c=%f, d=%f", a, b, c, d);

      if (c <= threshold)
      {
        RCLCPP_WARN(node_->get_logger(), "SF strategy plugin [%s] found a plane model with c too close to 0.", name_.c_str());
        grid_map[output_layer].setConstant(0.0);
        return;
      }

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
  };

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
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();

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
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){

      if (!last_imu_message_) {
        last_imu_message_ = msg;
        return;
      }

      // time sample = time of current msg - time of last msg
      auto time_sample = rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_message_->header.stamp);
      auto Ts = time_sample.seconds();
      auto Ts_over_T = Ts / this->node_->get_parameter(name_ + ".T").as_double();

      auto gridmap_frame_id = this->node_->get_parameter("gridmap_frame_id").as_string();

      geometry_msgs::msg::TransformStamped imu_transform;

      // try to get the transform from the IMU frame to the grid map frame using tf2
      try{
        imu_transform = tf_buffer_->lookupTransform(
          gridmap_frame_id,  
          msg->header.frame_id, 
          // get time transform relative to the time of the IMU message
          msg->header.stamp
        );        
      }
      catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->node_->get_logger(), "%s: %s", this->name_.c_str(), ex.what());
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

PLUGINLIB_EXPORT_CLASS(sfframework::GaussianRepulsion, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::ParabolicRest, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::WeightedSummation, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::PlaneInclination, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::PlaneInclinationFromImuLinear, sfframework::SFStrategy)