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
#include <visualization_msgs/msg/marker.hpp>

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    void publishDebugData(
      const std::string& frame_id,
      rclcpp::Time stamp,
      const std::vector<std::tuple<double, double, Eigen::Vector2d>>& buckets,
      int cicle_subs,
      double angle_resolution,
      double max_distance)
    {
      // construct polygon to be published
      geometry_msgs::msg::PolygonStamped polygon_msg;
      polygon_msg.header.frame_id = frame_id;
      polygon_msg.header.stamp = stamp;
      // without seer position
      for (const auto& bucket : buckets) {
        geometry_msgs::msg::Point32 vertex;
        vertex.x = std::get<2>(bucket)(0);
        vertex.y = std::get<2>(bucket)(1);
        vertex.z = 0.0;
        polygon_msg.polygon.points.push_back(vertex);
      }
      polygon_publisher_->publish(polygon_msg);

      // Publish PointCloud2 message
      sensor_msgs::msg::PointCloud2 pc_msg;
      pc_msg.header.frame_id = frame_id;
      pc_msg.header.stamp = stamp;

      sensor_msgs::PointCloud2Modifier modifier(pc_msg);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(cicle_subs);

      sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

      for (int i = 0; i < cicle_subs; ++i) {
        auto& p = std::get<2>(buckets[i]);
        *iter_x = static_cast<float>(p(0));
        *iter_y = static_cast<float>(p(1));
        *iter_z = 0.0f;
        ++iter_x; ++iter_y; ++iter_z;
      }
      pointcloud_publisher_->publish(pc_msg);
    }

    void onInitialize(grid_map::GridMap &grid_map) override
    {
      node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
      node_->declare_parameter(name_ + ".invert_selection", false);
      node_->declare_parameter(name_ + ".output_layer", "distance_field");
      node_->declare_parameter(name_ + ".seer_frame_id", "camera_link");
      node_->declare_parameter(name_ + ".circle_subdivisions", 360);
      node_->declare_parameter(name_ + ".robot_radius", 0.5);
      node_->declare_parameter(name_ + ".max_distance", 5.0);

      grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    
      // new debug gridmap publisher
      // debug_gridmap_publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("~/debug_gridmap", 10);

      // debug polygon publisher
      polygon_publisher_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("~/debug_polygon", 10);
      process_time_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("~/" + name_ + "/process_time", 10);
      pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug_pointcloud", 10);
      marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("~/debug_marker", 10);
    }
    void onProcess(PartitioningContext &context, grid_map::GridMap &grid_map) override
    {
      auto tstart = std::chrono::high_resolution_clock::now();

      auto input_tags = this->node_->get_parameter(name_ + ".input_tags").as_string_array();
      auto invert_selection = this->node_->get_parameter(name_ + ".invert_selection").as_bool();
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();
      auto seer_frame_id = this->node_->get_parameter(name_ + ".seer_frame_id").as_string();
      auto cicle_subs = this->node_->get_parameter(name_ + ".circle_subdivisions").as_int();
      auto robot_radius = this->node_->get_parameter(name_ + ".robot_radius").as_double();
      auto max_distance_from_seer = this->node_->get_parameter(name_ + ".max_distance").as_double();
      
      double epsilon = 1e-6;

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

      std::vector<std::tuple<double, double, Eigen::Vector2d>> angles_distances2_points;
      angles_distances2_points.reserve(o3d_pc->points_.size());
      for (const auto& pt : o3d_pc->points_) {
        double distance_sq = (pt.head<2>() - seer_position).squaredNorm();
        if (distance_sq < epsilon) {
          continue; // Skip points exactly on or extremely close to the sensor
        }
        double angle = std::atan2(pt(1) - seer_position(1), pt(0) - seer_position(0));
        angles_distances2_points.emplace_back(angle, distance_sq, pt.head<2>());
      }

      double max_distance_from_seer_sqrd = max_distance_from_seer * max_distance_from_seer;

      double angle_resolution = 2 * M_PI / cicle_subs;

      // subdivision "bucket" that actually contains only the required point
      std::vector<std::tuple<double, double, Eigen::Vector2d>> adp_subdivision_buckets(cicle_subs);

      // initialize polygon with points at the limit (circle)
      for(int i = 0; i < cicle_subs; i++){
        double angle = angle_resolution * i;
        std::get<0>(adp_subdivision_buckets[i]) = angle;
        std::get<1>(adp_subdivision_buckets[i]) = max_distance_from_seer_sqrd;
        std::get<2>(adp_subdivision_buckets[i]) = Eigen::Vector2d(seer_position(0) + std::cos(angle) * max_distance_from_seer, seer_position(1) + std::sin(angle) * max_distance_from_seer);
      }

      // for each bucket, get correct point
      for(const auto& pt : angles_distances2_points){
        double p_angle = std::get<0>(pt);
        if (p_angle < 0.0) {
          p_angle += 2.0 * M_PI;
        }
        double p_distance_sq = std::get<1>(pt);
        
        // this line defines in which bucket which point ends up
        int p_idx = static_cast<int>(p_angle / angle_resolution);
        if (p_idx >= cicle_subs) {
          p_idx = cicle_subs - 1;
        }

        double b_distance_sq = std::get<1>(adp_subdivision_buckets[p_idx]); 
      
        if(p_distance_sq < b_distance_sq){
          adp_subdivision_buckets[p_idx] = pt;
        }

      }

      // BPA inspired hole filling
      std::vector<int> removed_points_idx;
      for(int i = 0; i < cicle_subs; i++){
        const auto& p1 = adp_subdivision_buckets[i];
        double p1_distance_sq = std::get<1>(p1);

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
        int bucket_diff_int = static_cast<int>(std::ceil(bucket_diff));

        // propagate forward (positive angle)
        for (int j = bucket_diff_int; j > 0; j--) {
          int target_idx = (i + j) % cicle_subs;
          auto p2 = adp_subdivision_buckets[target_idx];

          if(std::get<1>(p2) >= max_distance_from_seer_sqrd - epsilon){
            continue;
          }


          auto v = (std::get<2>(p2) - std::get<2>(p1));
          auto d = v.norm();

          if(d > 2 * robot_radius - epsilon){
            continue;
          }

          // Find the midpoint between p1 and p2
          auto pm = (std::get<2>(p2) + std::get<2>(p1))/2;
          // Distance from midpoint to the circle's center
          double val = robot_radius * robot_radius - (d*d/4.0);
          auto h = std::sqrt(std::max(0.0, val));
          auto v_perp = Eigen::Vector2d(-v(1), v(0)).normalized(); // Perpendicular vector to (p2-p1) in XY plane
          // Calculate dot product of vector from seer_position to midpoint with v_perp
          // This determines which side of the line (p1, p2) the seer is, guiding the choice of circle center.
          auto dot_k = (pm - seer_position).dot(v_perp);

          // Select the circle center that is closest to the origin
          Eigen::Vector2d c;
          if(dot_k < 0){
            c = pm + h*v_perp;
          }
          else{
            c = pm - h*v_perp;
          }

          // check if there are no other points inside the circle
          // and the pair of points is closest than other points
          bool should_continue = false;
          // std::string rejection_reason = "";
          for(int step = 1; step < j; step++){
            int target_idx_k = (i + step) % cicle_subs;
            auto p3 = adp_subdivision_buckets[target_idx_k];

            auto p3_distance_from_seer_sq = std::get<1>(p3);
            
            double t = static_cast<double>(step) / j;
            Eigen::Vector2d interpolated_p = (1 - t) * std::get<2>(p1) + t * std::get<2>(p2);
            if (p3_distance_from_seer_sq < (interpolated_p - seer_position).squaredNorm()){
              should_continue = true;
              // rejection_reason = "p3 is closer to the sensor than the interpolated line segment";
              break;
            }

            auto vec_c = (std::get<2>(p3) - c);
            auto dist_c = vec_c.norm();
            if(dist_c < robot_radius){
              should_continue = true;
              // rejection_reason = "p3 is inside the proposed circle";
              break;
            }
          }

          if(should_continue){
            // RCLCPP_INFO(node_->get_logger(), "Circle rejected between i=%d and j=%d: %s", i, j, rejection_reason.c_str());
            
            // visualization_msgs::msg::Marker marker;
            // marker.header.frame_id = grid_map.getFrameId();
            // marker.header.stamp = rclcpp::Time(context.header.stamp);
            // marker.ns = "rejected_circles";
            // marker.id = 0;
            // marker.type = visualization_msgs::msg::Marker::CYLINDER;
            // marker.action = visualization_msgs::msg::Marker::ADD;
            // marker.pose.position.x = c.x();
            // marker.pose.position.y = c.y();
            // marker.pose.position.z = 0.05;
            // marker.pose.orientation.w = 1.0;
            // marker.scale.x = robot_radius * 2.0;
            // marker.scale.y = robot_radius * 2.0;
            // marker.scale.z = 0.05;
            // marker.color.r = 1.0;
            // marker.color.g = 0.0;
            // marker.color.b = 0.0;
            // marker.color.a = 0.5;
            
            // marker_publisher_->publish(marker);
            
            // publishDebugData(grid_map.getFrameId(), rclcpp::Time(context.header.stamp), 
            //       adp_subdivision_buckets, cicle_subs, angle_resolution, 
            //       max_distance_from_seer);
            
            // char x;
            // std::cin >> x;
            continue;
          }

          // fill from i to target_idx with the line segment p1 -> p2
          // (by marking these indices for removal)
          for (int step = 1; step < j; step++) {
            int target_idx_k = (i + step) % cicle_subs;
            removed_points_idx.push_back(target_idx_k);
          }
          // mark 

          // Skip ahead over the newly filled virtual points to prevent recursive shadow casting
          i += j - 1;
          break;
        }

      }

      // remove points from list of points
      std::sort(removed_points_idx.begin(), removed_points_idx.end());
      removed_points_idx.erase(std::unique(removed_points_idx.begin(), removed_points_idx.end()), removed_points_idx.end());
      for(int i = removed_points_idx.size() - 1; i >= 0; i--){
        adp_subdivision_buckets.erase(adp_subdivision_buckets.begin() + removed_points_idx[i]);
      }



      publishDebugData(grid_map.getFrameId(), rclcpp::Time(context.header.stamp), 
                       adp_subdivision_buckets, adp_subdivision_buckets.size(), angle_resolution, 
                       max_distance_from_seer);

      auto tend = std::chrono::high_resolution_clock::now();
      auto duration = tend - tstart;
      std_msgs::msg::Float64 time_msg;
      time_msg.data = duration.count() * epsilon;
      process_time_publisher_->publish(time_msg);
      // wait for next frame for debug
      // char x;
      // std::cin >> x;
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