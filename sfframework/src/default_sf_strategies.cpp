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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

#if __has_include("tf2_sensor_msgs/tf2_sensor_msgs.hpp")
  #include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#elif __has_include("imu_transformer/tf2_sensor_msgs.h")
  #include "imu_transformer/tf2_sensor_msgs.h"
#endif

namespace sfframework
{
  double epsilon = 1e-6;

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
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr process_time_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;

    void publishDebugData(
      const std::string& frame_id,
      rclcpp::Time stamp,
      const std::vector<std::tuple<double, double, Eigen::Vector2d>>& radial_buckets,
      int circle_subdivisions,
      double angle_resolution,
      double max_distance)
    {
      geometry_msgs::msg::PolygonStamped polygon_msg;
      polygon_msg.header.frame_id = frame_id;
      polygon_msg.header.stamp = stamp;
      
      for (const auto& bucket : radial_buckets) {
        geometry_msgs::msg::Point32 vertex;
        vertex.x = std::get<2>(bucket)(0);
        vertex.y = std::get<2>(bucket)(1);
        vertex.z = 0.0;
        polygon_msg.polygon.points.push_back(vertex);
      }
      polygon_publisher_->publish(polygon_msg);

      sensor_msgs::msg::PointCloud2 pc_msg;
      pc_msg.header.frame_id = frame_id;
      pc_msg.header.stamp = stamp;

      sensor_msgs::PointCloud2Modifier modifier(pc_msg);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(radial_buckets.size());

      sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

      for (long unsigned int i = 0; i < radial_buckets.size(); ++i) {
        auto& p = std::get<2>(radial_buckets[i]);
        *iter_x = static_cast<float>(p(0));
        *iter_y = static_cast<float>(p(1));
        *iter_z = 0.0f;
        ++iter_x; ++iter_y; ++iter_z;
      }
      pointcloud_publisher_->publish(pc_msg);

      // Publish LaserScan message
      sensor_msgs::msg::LaserScan scan_msg;
      scan_msg.header.frame_id = frame_id;
      scan_msg.header.stamp = stamp;
      scan_msg.angle_min = 0.0;
      scan_msg.angle_max = 2 * M_PI;
      scan_msg.angle_increment = angle_resolution;
      scan_msg.range_min = 0.0;
      scan_msg.range_max = max_distance;

      scan_msg.ranges.resize(circle_subdivisions, static_cast<float>(max_distance));
      scan_msg.intensities.resize(circle_subdivisions, 0.0f);


      if (radial_buckets.empty()) {
        laserscan_publisher_->publish(scan_msg);
        return;
      }

      int num_buckets = radial_buckets.size();
      int point_idx = 0; // Index for radial_buckets

      for (int i = 0; i < circle_subdivisions; ++i) {
        double i_angle = 2 * M_PI * i / circle_subdivisions;
        
        int point1_idx = point_idx % num_buckets;
        int point2_idx = (point_idx + 1) % num_buckets;

        auto radial1 = radial_buckets[point1_idx];
        auto radial2 = radial_buckets[point2_idx];
        
        double angle1 = std::get<0>(radial1);
        double angle2 = std::get<0>(radial2);

        // Handle wrap-around at the 2*PI boundary
        if(angle2 < angle1){
          angle2 += 2 * M_PI;
        }

        // If the current angle surpasses angle2, advance the bucket points and try again
        if (i_angle > angle2 && i_angle < angle1 + 2 * M_PI) {
          point_idx++;
          i--; 
          continue;
        }

        // RCLCPP_INFO(node_->get_logger(), "point_idx: %d, i: %d, i_angle: %f, angle1: %f, angle2: %f", point_idx, i, i_angle, angle1, angle2);

        double p1_dist_sq = std::get<1>(radial1);
        double p2_dist_sq = std::get<1>(radial2);

        // check if both of the points is too close to max_distance threshold (squared!)
        double max_dist_sq = max_distance * max_distance;
        if(p1_dist_sq >= max_dist_sq - 1e-6 && p2_dist_sq >= max_dist_sq - 1e-6) {
          scan_msg.intensities[i] = point1_idx;
          continue; // leaves the default max_distance in the range
        }

        // Using closest point distance (without interpolation for now)
        // scan_msg.ranges[i] = std::get<2>(radial1).norm();
        // scan_msg.intensities[i] = point1_idx;

        // determine where in the line segment from p1 to p2 
        // the ray would cross
        // u is our directional vector (cos(theta), sin(theta))
        auto u = Eigen::Vector2d(cos(i_angle), sin(i_angle));
        
        // points 1 and 2
        auto p1 = std::get<2>(radial1);
        auto p2 = std::get<2>(radial2);

        // line segment direction
        auto d = p2 - p1;

        // denominator: u x d
        // u(0)*d(1) - u(1)*d(0) is mathematically u cross d
        double denominator = u(0) * d(1) - u(1) * d(0);

        // Ensure the ray and the segment are not parallel
        if (std::abs(denominator) > 1e-9) {
          // Calculate distance using 2D cross product scalar optimization
          // distance = (p1 x p2) / (u x d)
          // Since Eigen::Vector2d doesn't natively have a scalar cross product, we expand it:
          double distance = (p1.x() * p2.y() - p1.y() * p2.x()) / denominator;

          if (distance > 0) {
            scan_msg.ranges[i] = distance;
          }
        }
        
        scan_msg.intensities[i] = point1_idx;

      }

      laserscan_publisher_->publish(scan_msg);
      // char x;
      // std::cin >> x;
    }

    
    void onInitialize(grid_map::GridMap &grid_map) override
    {
      node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
      node_->declare_parameter(name_ + ".invert_selection", false);
      node_->declare_parameter(name_ + ".output_layer", "distance_field");
      node_->declare_parameter(name_ + ".circle_subdivisions", 360);
      node_->declare_parameter(name_ + ".robot_radius", 0.5);
      node_->declare_parameter(name_ + ".max_distance", 5.0);

      grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    
      polygon_publisher_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("~/debug_polygon", 10);
      process_time_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("~/" + name_ + "/process_time", 10);
      pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug_pointcloud", 10);
      marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("~/debug_marker", 10);
      laserscan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("~/debug_laserscan", 10);
    }
    void onProcess(PartitioningContext &context, grid_map::GridMap &grid_map) override
    {
      auto tstart = std::chrono::high_resolution_clock::now();

      auto input_tags = this->node_->get_parameter(name_ + ".input_tags").as_string_array();
      auto invert_selection = this->node_->get_parameter(name_ + ".invert_selection").as_bool();
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();
      auto circle_subdivisions = this->node_->get_parameter(name_ + ".circle_subdivisions").as_int();
      auto robot_radius = this->node_->get_parameter(name_ + ".robot_radius").as_double();
      auto max_distance_from_seer = this->node_->get_parameter(name_ + ".max_distance").as_double();
      
      grid_map[output_layer].setConstant(std::numeric_limits<float>::quiet_NaN());

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

      std::vector<std::tuple<double, double, Eigen::Vector2d>> point_polar_data;
      point_polar_data.reserve(o3d_pc->points_.size());
      for (const auto& pt : o3d_pc->points_) {
        double distance_sq = pt.head<2>().squaredNorm();
        if (distance_sq < epsilon) {
          // skip points exactly on or extremely close to the sensor
          continue;
        }
        double angle = std::atan2(pt(1), pt(0));
        point_polar_data.emplace_back(angle, distance_sq, pt.head<2>());
      }

      double max_distance_from_seer_sqrd = max_distance_from_seer * max_distance_from_seer;

      double angle_resolution = 2 * M_PI / circle_subdivisions;

      // maintain a discrete set of rays originating from the sensor, keeping the closest point per ray
      std::vector<std::tuple<double, double, Eigen::Vector2d>> radial_buckets(circle_subdivisions);

      for(int i = 0; i < circle_subdivisions; i++){
        double angle = angle_resolution * i;
        std::get<0>(radial_buckets[i]) = angle;
        std::get<1>(radial_buckets[i]) = max_distance_from_seer_sqrd;
        std::get<2>(radial_buckets[i]) = Eigen::Vector2d(std::cos(angle) * max_distance_from_seer, std::sin(angle) * max_distance_from_seer);
      }

      for(auto pt : point_polar_data){ // Remove 'const &' to allow modification
        double p_angle = std::get<0>(pt);
        if (p_angle < 0.0) {
          p_angle += 2.0 * M_PI;
          std::get<0>(pt) = p_angle;
        }
        double p_distance_sq = std::get<1>(pt);
        
        int bucket_idx = static_cast<int>(p_angle / angle_resolution);
        if (bucket_idx >= circle_subdivisions) {
          bucket_idx = circle_subdivisions - 1;
        }

        double bucket_distance_sq = std::get<1>(radial_buckets[bucket_idx]); 
      
        if(p_distance_sq < bucket_distance_sq){
          radial_buckets[bucket_idx] = pt; // Now pt has the corrected 0 to 2pi angle
        }
      }

      // BPA inspired hole filling
      std::vector<int> removed_points_idx;
      for(int i = 0; i < circle_subdivisions; i++){
        const auto& point1 = radial_buckets[i];
        double p1_distance_sq = std::get<1>(point1);

        if (p1_distance_sq >= max_distance_from_seer_sqrd - epsilon) {
          continue;
        }

        // Calculate the angular width of the shadow cast by this point
        // sin(angle_diff/2) = (robot_radius/2) / distance  
        // => angle_diff/2 = asin(robot_radius / (2*distance))
        // => angle_diff = 2 * asin(robot_radius / (2 * distance)) (clip to) pi/2
        // => bucket_diff = angle_diff / angle_resolution
        double ratio = robot_radius / (2*std::sqrt(p1_distance_sq));
        double bucket_diff = (ratio >= 1.0) ? (M_PI / 2.0) / angle_resolution : (2 * std::asin(ratio)) / angle_resolution;
        int num_buckets_covered = static_cast<int>(std::ceil(bucket_diff));

        // attempt to connect the current point to a subsequent point within the robot radius limits
        for (int j = num_buckets_covered; j > 0; j--) {
          int target_idx = (i + j) % circle_subdivisions;
          auto point2 = radial_buckets[target_idx];

          if(std::get<1>(point2) >= max_distance_from_seer_sqrd - epsilon){
            continue;
          }

          auto diff_vec = (std::get<2>(point2) - std::get<2>(point1));
          auto dist_between_points = diff_vec.norm();

          if(dist_between_points > 2 * robot_radius - epsilon){
            continue;
          }

          auto midpoint = (std::get<2>(point2) + std::get<2>(point1))/2;
          double val = robot_radius * robot_radius - (dist_between_points*dist_between_points/4.0);
          auto height_to_center = std::sqrt(std::max(0.0, val));
          auto perp_vec = Eigen::Vector2d(-diff_vec(1), diff_vec(0)).normalized(); 
          
          // determine which side of the segment faces the sensor to choose the correct circle center
          auto dot_k = midpoint.dot(perp_vec);

          Eigen::Vector2d circle_center;
          if(dot_k < 0){
            circle_center = midpoint + height_to_center*perp_vec;
          }
          else{
            circle_center = midpoint - height_to_center*perp_vec;
          }

          // validate that the proposed circle is empty and no intermediate points block the sensor
          bool should_continue = false;
          for(int step = 1; step < j; step++){
            int intermediate_idx = (i + step) % circle_subdivisions;
            auto intermediate_point = radial_buckets[intermediate_idx];

            auto p3_distance_from_seer_sq = std::get<1>(intermediate_point);
            
            double t_interpolation = static_cast<double>(step) / j;
            Eigen::Vector2d interpolated_p = (1 - t_interpolation) * std::get<2>(point1) + t_interpolation * std::get<2>(point2);
            if (p3_distance_from_seer_sq < interpolated_p.squaredNorm()){
              should_continue = true;
              break;
            }

            auto vec_from_center = (std::get<2>(intermediate_point) - circle_center);
            auto dist_from_center = vec_from_center.norm();
            if(dist_from_center < robot_radius){
              should_continue = true;
              break;
            }
          }

          if(should_continue){
            continue;
          }

          // filter out the occluded points contained within the filled gap
          for (int step = 1; step < j; step++) {
            int target_idx_k = (i + step) % circle_subdivisions;
            removed_points_idx.push_back(target_idx_k);
          }

          // skip ahead over the newly filled virtual points to prevent recursive shadow casting
          i += j - 1;
          break;
        }

      }

      std::sort(removed_points_idx.begin(), removed_points_idx.end());
      removed_points_idx.erase(std::unique(removed_points_idx.begin(), removed_points_idx.end()), removed_points_idx.end());
      for(int i = removed_points_idx.size() - 1; i >= 0; i--){
        radial_buckets.erase(radial_buckets.begin() + removed_points_idx[i]);
      }

      publishDebugData(grid_map.getFrameId(), rclcpp::Time(context.header.stamp), 
                       radial_buckets, circle_subdivisions, angle_resolution, 
                       max_distance_from_seer);

      auto tend = std::chrono::high_resolution_clock::now();
      auto duration = tend - tstart;
      std_msgs::msg::Float64 time_msg;
      time_msg.data = std::chrono::duration<double, std::milli>(duration).count();
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