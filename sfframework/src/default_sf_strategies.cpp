#include <pluginlib/class_list_macros.hpp>
#include "sfframework/sf_strategy.hpp"
#include "sfframework/sfgenerator_utils.hpp"
#include "sfframework/cluster_types.hpp"
#include <omp.h>
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Geometry>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <limits>
#include <chrono>
#include <std_msgs/msg/float64.hpp>

#if __has_include("tf2_sensor_msgs/tf2_sensor_msgs.hpp")
  #include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#elif __has_include("imu_transformer/tf2_sensor_msgs.h")
  #include "imu_transformer/tf2_sensor_msgs.h"
#endif

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
      auto distance_threshold_sqrd = sigma * 9.0;
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
            // Atomically update the cell with the maximum potential to prevent race conditions.
            float& cell_value = grid_map.at(output_layer, *iterator);
            float potential_f = static_cast<float>(potential);
            if (potential_f > cell_value) {
              #pragma omp critical
              if (potential_f > cell_value) cell_value = potential_f;
            }
          }
        }
      }
    }
  };

  class DistanceField1 : public SFStrategy
  {
    // rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr debug_gridmap_publisher_;
    //debug polygon publisher
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr process_time_publisher_;

    void onInitialize(grid_map::GridMap &grid_map) override
    {
      node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
      node_->declare_parameter(name_ + ".invert_selection", false);
      node_->declare_parameter(name_ + ".output_layer", "distance_field");
      node_->declare_parameter(name_ + ".seer_frame_id", "camera_link");

      grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    
      // new debug gridmap publisher
      // debug_gridmap_publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("~/debug_gridmap", 10);

      // debug polygon publisher
      polygon_publisher_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("~/debug_polygon", 10);
      process_time_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("~/" + name_ + "/process_time", 10);
    }
    void onProcess(PartitioningContext &context, grid_map::GridMap &grid_map) override
    {
      auto tstart = std::chrono::high_resolution_clock::now();

      auto input_tags = this->node_->get_parameter(name_ + ".input_tags").as_string_array();
      auto invert_selection = this->node_->get_parameter(name_ + ".invert_selection").as_bool();
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();
      auto seer_frame_id = this->node_->get_parameter(name_ + ".seer_frame_id").as_string();
      
      // get seer position in grid map frame
      geometry_msgs::msg::TransformStamped seer_transform;
      try {
        seer_transform = tf_buffer_->lookupTransform(
          grid_map.getFrameId(),
          seer_frame_id,
          rclcpp::Time(context.header.stamp)
        );
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "%s: %s", this->name_.c_str(), ex.what());
        return;
      }
      // Reset the layer to zero before accumulating new values
      // grid_map[output_layer].setConstant(std::numeric_limits<float>::infinity());
      // grid_map[output_layer].setConstant(0.0);
      grid_map[output_layer].setConstant(std::numeric_limits<float>::quiet_NaN());

      // squish z coordinate of seer position since grid map is 2D
      Eigen::Vector2d seer_position(seer_transform.transform.translation.x, seer_transform.transform.translation.y);

      // attempt to get point cloud from context
      std::shared_ptr<const open3d::geometry::PointCloud> o3d_pc;
      try
      {
        o3d_pc = sfframework::SFGeneratorUtils::cloud_from_tags(
          context,
          input_tags,
          invert_selection
        );
      }
      catch (const std::out_of_range &e)
      {
        RCLCPP_WARN(node_->get_logger(), "SF strategy plugin [%s] tried to find a list of clusters at a tag but it is missing. Skipping...", name_.c_str());
        return;
      }

      // calculate point angles relative to seer position

      std::vector<std::tuple<double, double, Eigen::Vector3d>> angles_distances2_points;
      angles_distances2_points.reserve(o3d_pc->points_.size());
      for (const auto& pt : o3d_pc->points_) {
        double distance = (pt.head<2>() - seer_position).squaredNorm();
        if (distance < 1e-6) {
          continue; // Skip points exactly on or extremely close to the sensor
        }
        double angle = std::atan2(pt(1) - seer_position(1), pt(0) - seer_position(0));
        angles_distances2_points.emplace_back(angle, distance, pt);
      }
      // sort points by angle
      std::sort(angles_distances2_points.begin(), angles_distances2_points.end(), [](const std::tuple<double, double, Eigen::Vector3d>& a, const std::tuple<double, double, Eigen::Vector3d>& b) {
        return std::get<0>(a) < std::get<0>(b);
      });


      // various parameters used to the filter 
      // basically the minimum feature size = 2 * grid cell size (Nyquist?)
      double min_distance_diff_sqrd = 4 * grid_map.getResolution() * grid_map.getResolution();
      // defined as maximum distance from seer to one of the map corners
      // points further than this distance can be "ignored"
      grid_map::Position map_center = grid_map.getPosition();
      grid_map::Length map_length = grid_map.getLength();
      double max_distance_from_seer_sqrd = 0.0;
      for (double sign_x : {-1.0, 1.0}) {
        for (double sign_y : {-1.0, 1.0}) {
          Eigen::Vector2d corner(map_center(0) + sign_x * map_length(0) / 2.0, map_center(1) + sign_y * map_length(1) / 2.0);
          max_distance_from_seer_sqrd = std::max(max_distance_from_seer_sqrd, (corner - seer_position).squaredNorm());
        }
      }
      double max_distance_from_seer = std::sqrt(max_distance_from_seer_sqrd);

      // calculate minimum angle required to not skip a cell at the furthest point
      double angle_resolution = std::atan2(grid_map.getResolution()*2, std::sqrt(max_distance_from_seer_sqrd));

      // RCLCPP_INFO(node_->get_logger(), "DistanceField1 strategy parameters: min_distance_diff=%f, max_distance_from_seer=%f, angle_resolution=%f", std::sqrt(min_distance_diff_sqrd), std::sqrt(max_distance_from_seer_sqrd), angle_resolution);

      // sort of complex point filter
      std::vector<std::tuple<double, double, Eigen::Vector2d>> filtered_points;
      
      filtered_points.reserve(angles_distances2_points.size());

      for (const auto& adp_tuple : angles_distances2_points) {
        double current_angle = std::get<0>(adp_tuple);
        double current_dist_sqrd = std::get<1>(adp_tuple);
        Eigen::Vector2d current_pt = std::get<2>(adp_tuple).head<2>();

        // project point to circle if distance is too big
        if (current_dist_sqrd > max_distance_from_seer_sqrd) {
          Eigen::Vector2d direction = (current_pt - seer_position).normalized();
          current_pt = seer_position + direction * max_distance_from_seer;
          current_dist_sqrd = max_distance_from_seer_sqrd;
        }

        if (filtered_points.empty()) {
          filtered_points.emplace_back(current_angle, current_dist_sqrd, current_pt);
          continue;
        } 
        auto last_tuple = filtered_points.back();

        // remove points based on angle difference (relative to seer) between each other
        double angle_diff = std::abs(current_angle - std::get<0>(last_tuple));
        if (angle_diff > M_PI) {
          angle_diff = 2.0 * M_PI - angle_diff;
        }
        
        // if angle difference is smaller than the resolution, only keep the closest point
        if(angle_diff <= angle_resolution){
          if (current_dist_sqrd < std::get<1>(last_tuple)) {
            filtered_points.back() = std::make_tuple(std::get<0>(last_tuple), current_dist_sqrd, current_pt);
          }
          continue;
        }
        // if angle difference exceeds the resolution and 
        // the two points are further from each other than the minimum 
        // distance difference, place points between them
        // projected on the circle 
        // also add the current point to the list if it passed the filters
        else if(current_dist_sqrd > min_distance_diff_sqrd){
          double angle_to_place = std::get<0>(last_tuple) + angle_resolution;
          while (angle_to_place < current_angle) {
            Eigen::Vector2d direction(std::cos(angle_to_place), std::sin(angle_to_place));
            Eigen::Vector2d new_pt = seer_position + direction * std::sqrt(current_dist_sqrd);
            filtered_points.emplace_back(angle_to_place, current_dist_sqrd, new_pt);
            angle_to_place += angle_resolution;
          }
        }
        // if the point passed the filters, add it to the list
        filtered_points.emplace_back(current_angle, current_dist_sqrd, current_pt);
      }
      
      // construct polygon to be published
      geometry_msgs::msg::PolygonStamped polygon_msg;
      polygon_msg.header.frame_id = grid_map.getFrameId();
      polygon_msg.header.stamp = rclcpp::Time(context.header.stamp);
      // without seer position
      for (const auto& tuple : filtered_points) {
        const auto& pt = std::get<2>(tuple);
        geometry_msgs::msg::Point32 vertex;
        vertex.x = pt(0);
        vertex.y = pt(1);
        vertex.z = 0.0;
        polygon_msg.polygon.points.push_back(vertex);
      }
      polygon_publisher_->publish(polygon_msg);

      auto tend = std::chrono::high_resolution_clock::now();
      auto duration = tend - tstart;
      std_msgs::msg::Float64 time_msg;
      time_msg.data = duration.count() * 1e-6;
      process_time_publisher_->publish(time_msg);
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
PLUGINLIB_EXPORT_CLASS(sfframework::DistanceField1, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::ParabolicRest, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::WeightedSummation, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::PlaneInclination, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::PlaneInclinationFromImuLinear, sfframework::SFStrategy)