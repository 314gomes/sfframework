#include <pluginlib/class_list_macros.hpp>

#include "sfframework/sf_strategy.hpp"
#include "sfframework/sfgenerator_utils.hpp"

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Polyline_simplification_2/Vertex_base_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Polyline_simplification_2/Stop_above_cost_threshold.h>

#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>
#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace sfframework
{
struct RadialBucketPoint
{
  double angle = 0.0;
  double distance_sq = 0.0;
  Eigen::Vector2d point = Eigen::Vector2d::Zero();
  int metadata = 0;
  int dbscan_label = -1;
  // Selected cluster for this slice: label of the cluster whose centroid
  // is closest to the sensor origin (in XY). -1 if none.
  int selected_dbscan_label = -1;
  // Centroid of the selected cluster (3D) and full 3x3 covariance.
  Eigen::Vector3d selected_dbscan_centroid = Eigen::Vector3d::Zero();
  Eigen::Matrix3d selected_dbscan_covariance = Eigen::Matrix3d::Zero();
  Eigen::Vector3d selected_dbscan_eigenvalues = Eigen::Vector3d::Zero();
  Eigen::Matrix3d selected_dbscan_eigenvectors = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d selected_dbscan_precision = Eigen::Matrix3d::Zero();
};

class RadialBucketMahalanobisCost {
public:
  RadialBucketMahalanobisCost() = default;

  RadialBucketMahalanobisCost(
    const std::vector<RadialBucketPoint> *buckets)
  : buckets_(buckets)
  {
  }

  template <typename Constrained_triangulation, typename VertexIterator>
  boost::optional<typename Constrained_triangulation::Geom_traits::FT>
  operator()(Constrained_triangulation &ct, VertexIterator u) const
  {
    using FT = typename Constrained_triangulation::Geom_traits::FT;
    // `u` is an iterator over the vertices in the current constraint; it
    // points to the vertex considered for removal. We must compute the cost
    // as the squared distance from that vertex to the segment formed by its
    // predecessor and successor along the constraint.

    // Dereference to obtain the current vertex handle
    auto vh = *u;
    if (ct.is_infinite(vh)) return boost::optional<FT>();

    // Obtain previous and next iterators (bidirectional)
    VertexIterator it_prev = u;
    VertexIterator it_next = u;
    --it_prev;
    ++it_next;

    auto vh_prev = *it_prev;
    auto vh_next = *it_next;

    if (ct.is_infinite(vh_prev) || ct.is_infinite(vh_next)) {
      return boost::optional<FT>();
    }

    // Extract points and convert to double for arithmetic
    auto p_mid = vh->point();
    auto p1 = vh_prev->point();
    auto p3 = vh_next->point();

    Eigen::Vector3d p1_eig(CGAL::to_double(p1.x()), CGAL::to_double(p1.y()), 0.0);
    Eigen::Vector3d p_mid_eig(CGAL::to_double(p_mid.x()), CGAL::to_double(p_mid.y()), 0.0);
    Eigen::Vector3d p3_eig(CGAL::to_double(p3.x()), CGAL::to_double(p3.y()), 0.0);

    auto line_vec = p3_eig - p1_eig;

    auto point_vec = p_mid_eig - p1_eig;

    auto projection_scalar = point_vec.dot(line_vec) / line_vec.squaredNorm();

    auto projection_point = p1_eig + projection_scalar * line_vec;

    auto distance_vector = projection_point - p_mid_eig;

    double mahalanobis_distance_squared = std::numeric_limits<double>::infinity();
    bool has_eigendecomposition = false;
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Zero();
    std::size_t bucket_idx = 0;
    const RadialBucketPoint *bucket = nullptr;
    if (buckets_) {
      bucket_idx = vh->info();
      if (bucket_idx < buckets_->size()) {
        bucket = &(*buckets_)[bucket_idx];
        eigenvalues = bucket->selected_dbscan_eigenvalues;
        eigenvectors = bucket->selected_dbscan_eigenvectors;
        has_eigendecomposition = eigenvalues.allFinite() && eigenvectors.allFinite();
        if (has_eigendecomposition) {
          constexpr double kMinEigenvalue = 1e-12;
          double candidate = 0.0;
          // Use truncation (Moore-Penrose pseudo-inverse): ignore near-zero eigenvalues.
          for (int i = 0; i < 3; ++i) {
            const double lambda = eigenvalues[i];
            if (lambda <= kMinEigenvalue) {
              continue;
            }
            const double projection = distance_vector.dot(eigenvectors.col(i));
            candidate += (projection * projection) / lambda;
          }
          if (std::isfinite(candidate)) {
            mahalanobis_distance_squared = candidate;
          }
        }
      }
    }

    if (mahalanobis_distance_squared < 0.0) {
      mahalanobis_distance_squared = 0.0;
    }
    const double mahalanobis_distance = std::sqrt(mahalanobis_distance_squared);

    return boost::optional<FT>(static_cast<FT>(mahalanobis_distance));
  }

private:
  const std::vector<RadialBucketPoint> *buckets_ = nullptr;
};

double epsilon = 1e-6;

struct RadialSlicePoint
{
  double angle = 0.0;
  double distance_sq = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  int metadata = 0;
  int dbscan_label = -1;
};

class CompressedStochasticPolygon : public SFStrategy
{
  struct ProcessParams
  {
    std::vector<std::string> input_tags;
    bool invert_selection = false;
    std::string output_layer;
    int circle_subdivisions = 0;
    double robot_radius = 0.0;
    double max_distance_from_seer = 0.0;
    double slice_dbscan_eps = 0.0;
    int slice_dbscan_min_points = 0;
    double simplification_mahalanobis_error_threshold = 0.0;
    bool covariance_markers_use_actual_z = false;
    std::vector<int> publish_metadata_filter;
  };

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr process_time_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_multiarray_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr distances_multiarray_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;

  ProcessParams loadProcessParams() const
  {
    ProcessParams params;
    params.input_tags = node_->get_parameter(name_ + ".input_tags").as_string_array();
    params.invert_selection = node_->get_parameter(name_ + ".invert_selection").as_bool();
    params.output_layer = node_->get_parameter(name_ + ".output_layer").as_string();
    params.circle_subdivisions = node_->get_parameter(name_ + ".circle_subdivisions").as_int();
    params.robot_radius = node_->get_parameter(name_ + ".robot_radius").as_double();
    params.max_distance_from_seer = node_->get_parameter(name_ + ".max_distance").as_double();
    params.slice_dbscan_eps = node_->get_parameter(name_ + ".slice_dbscan_eps").as_double();
    params.slice_dbscan_min_points = node_->get_parameter(name_ + ".slice_dbscan_min_points").as_int();
    params.simplification_mahalanobis_error_threshold =
      node_->get_parameter(name_ + ".simplification_mahalanobis_error_threshold").as_double();
    params.covariance_markers_use_actual_z =
      node_->get_parameter(name_ + ".covariance_markers_use_actual_z").as_bool();

    const auto publish_metadata_filter_param =
      node_->get_parameter(name_ + ".debug_publish_metadata_filter").as_integer_array();
    params.publish_metadata_filter.reserve(publish_metadata_filter_param.size());
    for (const auto metadata : publish_metadata_filter_param) {
      params.publish_metadata_filter.push_back(static_cast<int>(metadata));
    }

    return params;
  }

  std::vector<RadialSlicePoint> buildPolarData(
    const open3d::geometry::PointCloud &pointcloud) const
  {
    std::vector<RadialSlicePoint> point_polar_data;
    point_polar_data.reserve(pointcloud.points_.size());
    for (const auto &pt : pointcloud.points_) {
      double distance_sq = pt.head<2>().squaredNorm();
      if (distance_sq < epsilon) {
        // skip points exactly on or extremely close to the sensor
        continue;
      }
      double angle = std::atan2(pt(1), pt(0));
      point_polar_data.push_back({angle, distance_sq, pt});
    }
    return point_polar_data;
  }

  std::vector<std::vector<RadialSlicePoint>> bucketizePolarData(
    const std::vector<RadialSlicePoint> &point_polar_data,
    int circle_subdivisions,
    double angle_resolution) const
  {
    // maintain a discrete set of rays originating from the sensor, keeping ALL points per ray
    // Stores full 3D points
    std::vector<std::vector<RadialSlicePoint>> radial_buckets_multiarray(circle_subdivisions);

    // collect all points, preserving all points per ray
    for (auto pt : point_polar_data) {
      double p_angle = pt.angle;
      if (p_angle < 0.0) {
        p_angle += 2.0 * M_PI;
        pt.angle = p_angle;
      }

      int bucket_idx = static_cast<int>(p_angle / angle_resolution);
      if (bucket_idx >= circle_subdivisions) {
        bucket_idx = circle_subdivisions - 1;
      }

      // Add ALL points to the corresponding ray bucket (storing full 3D point)
      radial_buckets_multiarray[bucket_idx].push_back(pt);
    }

    return radial_buckets_multiarray;
  }

  void runDbscanOnBuckets(
    std::vector<std::vector<RadialSlicePoint>> &radial_buckets_multiarray,
    double slice_dbscan_eps,
    int slice_dbscan_min_points) const
  {
    // Run DBSCAN independently on each angular slice so segmentation stays local
    // to the slice and does not affect the existing geometric pipeline.
    for (auto &bucket : radial_buckets_multiarray) {
      if (bucket.empty()) {
        continue;
      }

      open3d::geometry::PointCloud bucket_cloud;
      bucket_cloud.points_.reserve(bucket.size());
      for (const auto &point : bucket) {
        bucket_cloud.points_.push_back(point.point);
      }

      const auto labels = bucket_cloud.ClusterDBSCAN(
        slice_dbscan_eps,
        slice_dbscan_min_points,
        false);

      for (std::size_t i = 0; i < bucket.size() && i < labels.size(); ++i) {
        bucket[i].dbscan_label = labels[i];
      }
    }
  }

  void selectClosestClusters(
    const std::vector<std::vector<RadialSlicePoint>> &radial_buckets_multiarray,
    std::vector<int> &selected_label_per_bucket,
    std::vector<Eigen::Vector3d> &selected_centroid_per_bucket,
    std::vector<Eigen::Matrix3d> &selected_covariance_per_bucket) const
  {
    for (int bucket_idx = 0; bucket_idx < static_cast<int>(radial_buckets_multiarray.size()); ++bucket_idx) {
      const auto &bucket = radial_buckets_multiarray[bucket_idx];
      if (bucket.empty()) continue;

      // Group points per DBSCAN label using Open3D point clouds
      std::unordered_map<int, open3d::geometry::PointCloud> label_clouds;
      for (const auto &tup : bucket) {
        int lab = tup.dbscan_label;
        if (lab < 0) continue;
        label_clouds[lab].points_.push_back(tup.point);
      }

      // Choose the label whose centroid (projected to XY) is closest to origin
      double best_dist = std::numeric_limits<double>::infinity();
      int best_label = -1;
      Eigen::Vector3d best_centroid3 = Eigen::Vector3d::Zero();
      Eigen::Matrix3d best_cov = Eigen::Matrix3d::Zero();

      for (auto &kv : label_clouds) {
        int lab = kv.first;
        auto &cloud = kv.second;
        if (cloud.points_.empty()) continue;

        // Compute centroid using Open3D helper if available
        Eigen::Vector3d centroid3 = Eigen::Vector3d::Zero();
        // prefer Open3D method when present
        centroid3 = cloud.GetCenter();

        double d = centroid3.head<2>().squaredNorm();
        if (d < best_dist) {
          best_dist = d;
          best_label = lab;
          best_centroid3 = centroid3;

          // compute full 3x3 covariance
          Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
          const double n = static_cast<double>(cloud.points_.size());
          if (n > 0) {
            for (const auto &p : cloud.points_) {
              Eigen::Vector3d diff = p - centroid3;
              cov += diff * diff.transpose();
            }
            cov /= n;
          }
          best_cov = cov;
        }
      }

      selected_label_per_bucket[bucket_idx] = best_label;
      selected_centroid_per_bucket[bucket_idx] = best_centroid3;
      selected_covariance_per_bucket[bucket_idx] = best_cov;
    }
  }

  std::vector<RadialBucketPoint> buildRadialBuckets(
    int circle_subdivisions,
    double angle_resolution,
    double max_distance_from_seer,
    double max_distance_from_seer_sqrd,
    const std::vector<std::vector<RadialSlicePoint>> &radial_buckets_multiarray,
    const std::vector<int> &selected_label_per_bucket,
    const std::vector<Eigen::Vector3d> &selected_centroid_per_bucket,
    const std::vector<Eigen::Matrix3d> &selected_covariance_per_bucket) const
  {
    std::vector<RadialBucketPoint> radial_buckets(circle_subdivisions);

    for (int i = 0; i < circle_subdivisions; i++) {
      // fill the buckets with the preliminary values (circle)
      double angle = angle_resolution * i;
      radial_buckets[i].angle = angle;
      radial_buckets[i].distance_sq = max_distance_from_seer_sqrd;
      radial_buckets[i].point = Eigen::Vector2d(
        std::cos(angle) * max_distance_from_seer,
        std::sin(angle) * max_distance_from_seer);
      radial_buckets[i].metadata = -2;
      radial_buckets[i].dbscan_label = -1;
      radial_buckets[i].selected_dbscan_label = -1;
      radial_buckets[i].selected_dbscan_centroid = Eigen::Vector3d::Zero();
      radial_buckets[i].selected_dbscan_covariance = Eigen::Matrix3d::Zero();
      radial_buckets[i].selected_dbscan_eigenvalues = Eigen::Vector3d::Zero();
      radial_buckets[i].selected_dbscan_eigenvectors = Eigen::Matrix3d::Zero();
      radial_buckets[i].selected_dbscan_precision = Eigen::Matrix3d::Zero();

      if (!radial_buckets_multiarray[i].empty()) {
        // Use the per-slice selected cluster label (closest centroid) instead
        // of the individual point label. The selected_label_per_bucket and
        // selected_centroid_per_bucket vectors were computed earlier.
        radial_buckets[i].dbscan_label = selected_label_per_bucket[i];
        radial_buckets[i].selected_dbscan_label = selected_label_per_bucket[i];
        radial_buckets[i].selected_dbscan_centroid = selected_centroid_per_bucket[i];

        // Keep the bucket angle tied to the ray index and only accept a
        // selected centroid when DBSCAN produced a valid per-slice label.
        if (radial_buckets[i].selected_dbscan_label >= 0) {
          radial_buckets[i].point = radial_buckets[i].selected_dbscan_centroid.head<2>();
          radial_buckets[i].distance_sq = radial_buckets[i].point.squaredNorm();
          radial_buckets[i].metadata = 0;
          radial_buckets[i].selected_dbscan_covariance = selected_covariance_per_bucket[i];
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
            radial_buckets[i].selected_dbscan_covariance);
          if (solver.info() == Eigen::Success) {
            Eigen::Vector3d evals = solver.eigenvalues();
            Eigen::Matrix3d evecs = solver.eigenvectors();
            for (int k = 0; k < 3; ++k) {
              if (evals[k] < 0.0) {
                evals[k] = 0.0;
              }
            }
            radial_buckets[i].selected_dbscan_eigenvalues = evals;
            radial_buckets[i].selected_dbscan_eigenvectors = evecs;
          }
        }
      }
    }

    return radial_buckets;
  }

  void publishPointcloudMultiarray(
    const grid_map::GridMap &grid_map,
    const PartitioningContext &context,
    int circle_subdivisions,
    const std::vector<std::vector<RadialSlicePoint>> &radial_buckets_multiarray,
    const std::vector<int> &selected_label_per_bucket) const
  {
    // Count total points
    size_t total_points = 0;
    for (int i = 0; i < circle_subdivisions; ++i) total_points += radial_buckets_multiarray[i].size();

    if (total_points > 0 && pointcloud_multiarray_publisher_) {
      sensor_msgs::msg::PointCloud2 pc_msg;
      pc_msg.header.frame_id = grid_map.getFrameId();
      pc_msg.header.stamp = rclcpp::Time(context.header.stamp);

      sensor_msgs::PointCloud2Modifier modifier(pc_msg);
      modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "index", 1, sensor_msgs::msg::PointField::INT16);
      modifier.resize(total_points);

      sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
      sensor_msgs::PointCloud2Iterator<uint16_t> iter_intensity(pc_msg, "index");

      for (int bucket_idx = 0; bucket_idx < circle_subdivisions; ++bucket_idx) {
        for (const auto &tup : radial_buckets_multiarray[bucket_idx]) {
          const auto &p = tup.point;
          *iter_x = static_cast<float>(p(0));
          *iter_y = static_cast<float>(p(1));
          *iter_z = static_cast<float>(p(2));

          int sel_label = selected_label_per_bucket[bucket_idx];

          *iter_intensity = (static_cast<uint8_t>(bucket_idx) % 2) << 1;
          (*iter_intensity) += tup.dbscan_label == sel_label;

          ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
        }
      }

      pointcloud_multiarray_publisher_->publish(pc_msg);
    }
  }

  void publishCovarianceMarkers(
    const grid_map::GridMap &grid_map,
    const PartitioningContext &context,
    const std::vector<RadialBucketPoint> &radial_buckets,
    bool covariance_markers_use_actual_z) const
  {
    if (!marker_array_publisher_) {
      return;
    }

    visualization_msgs::msg::MarkerArray cov_markers;
    visualization_msgs::msg::Marker delete_all_cov;
    delete_all_cov.header.frame_id = grid_map.getFrameId();
    delete_all_cov.header.stamp = rclcpp::Time(context.header.stamp);
    delete_all_cov.ns = "covariance_ellipsoids";
    delete_all_cov.id = 0;
    delete_all_cov.action = visualization_msgs::msg::Marker::DELETEALL;
    cov_markers.markers.push_back(delete_all_cov);

    // Also clear previously published axes markers in a separate namespace
    visualization_msgs::msg::Marker delete_all_axes;
    delete_all_axes.header.frame_id = grid_map.getFrameId();
    delete_all_axes.header.stamp = rclcpp::Time(context.header.stamp);
    delete_all_axes.ns = "covariance_axes";
    delete_all_axes.id = 0;
    delete_all_axes.action = visualization_msgs::msg::Marker::DELETEALL;
    cov_markers.markers.push_back(delete_all_axes);

    int marker_id = 1;
    for (const auto &rb : radial_buckets) {
      if (rb.selected_dbscan_label < 0) continue;

      const Eigen::Matrix3d &cov = rb.selected_dbscan_covariance;
      // quick NaN check
      double trace = cov.trace();
      if (!std::isfinite(trace)) continue;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
      if (solver.info() != Eigen::Success) continue;

      Eigen::Vector3d evals = solver.eigenvalues();
      Eigen::Matrix3d evecs = solver.eigenvectors();

      // Clamp negative eigenvalues (numerical noise) to zero
      for (int k = 0; k < 3; ++k) if (evals[k] < 0.0) evals[k] = 0.0;

      // Convert eigenvalues (variance) to axis lengths. Use 2*sigma as visual scale.
      Eigen::Vector3d axes = 2.0 * evals.cwiseSqrt();
      constexpr double kMinAxis = 1e-6;
      for (int k = 0; k < 3; ++k) if (axes[k] < kMinAxis) axes[k] = kMinAxis;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = grid_map.getFrameId();
      m.header.stamp = rclcpp::Time(context.header.stamp);
      m.ns = "covariance_ellipsoids";
      m.id = marker_id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;

      const double marker_z = covariance_markers_use_actual_z
        ? rb.selected_dbscan_centroid.z()
        : 0.0;

      // Position
      m.pose.position.x = rb.selected_dbscan_centroid.x();
      m.pose.position.y = rb.selected_dbscan_centroid.y();
      m.pose.position.z = marker_z;

      // Orientation from eigenvectors (rotation matrix)
      Eigen::Quaterniond q(evecs);
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();
      m.pose.orientation.w = q.w();

      // Scale is the axis lengths
      m.scale.x = axes.x() * 2.0;
      m.scale.y = axes.y() * 2.0;
      m.scale.z = axes.z() * 2.0;

      // Visual style
      m.color.r = 0.0f;
      m.color.g = 0.0f;
      m.color.b = 1.0f;
      m.color.a = 0.5f;

      cov_markers.markers.push_back(m);

      // Publish principal axes as arrows in a separate namespace
      const Eigen::Vector3d center = rb.selected_dbscan_centroid;
      for (int ax = 0; ax < 3; ++ax) {
        Eigen::Vector3d dir = evecs.col(ax);
        double len = axes[ax];
        if (!std::isfinite(len) || len <= 0.0) continue;

        geometry_msgs::msg::Point p_start;
        geometry_msgs::msg::Point p_end;
        p_start.x = center.x();
        p_start.y = center.y();
        // p_start.z = center.z();
        p_start.z = marker_z;
        Eigen::Vector3d p_end_eig = center + dir.normalized() * (len);
        p_end.x = p_end_eig.x();
        p_end.y = p_end_eig.y();
        p_end.z = covariance_markers_use_actual_z ? p_end_eig.z() : p_end_eig.z() - center.z();

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = grid_map.getFrameId();
        arrow.header.stamp = rclcpp::Time(context.header.stamp);
        arrow.ns = "covariance_axes";
        arrow.id = marker_id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.points.clear();
        arrow.points.push_back(p_start);
        arrow.points.push_back(p_end);

        // Shaft diameter and head size scaled from axis length (with minima)
        double shaft = std::max(0.01, 0.04 * len);
        double head_diam = std::max(0.02, shaft * 2.0);
        double head_len = std::max(0.02, 0.1 * len);
        arrow.scale.x = static_cast<float>(shaft);
        arrow.scale.y = static_cast<float>(head_diam);
        arrow.scale.z = static_cast<float>(head_len);

        // Color axes: principal axes colored R,G,B
        if (ax == 0) { arrow.color.r = 1.0f; arrow.color.g = 0.0f; arrow.color.b = 0.0f; }
        else if (ax == 1) { arrow.color.r = 0.0f; arrow.color.g = 1.0f; arrow.color.b = 0.0f; }
        else { arrow.color.r = 0.0f; arrow.color.g = 0.0f; arrow.color.b = 1.0f; }
        arrow.color.a = 0.9f;

        cov_markers.markers.push_back(arrow);
      }
    }

    if (!cov_markers.markers.empty()) {
      marker_array_publisher_->publish(cov_markers);
    }
  }

  void fillOcclusionsBPA(
    std::vector<RadialBucketPoint> &radial_buckets,
    int circle_subdivisions,
    double angle_resolution,
    double max_distance_from_seer_sqrd,
    double robot_radius) const
  {
    // BPA inspired hole filling
    for (int i = 0; i < circle_subdivisions; i++) {
      const auto &point1 = radial_buckets[i];
      double p1_distance_sq = point1.distance_sq;

      if (p1_distance_sq >= max_distance_from_seer_sqrd - epsilon) {
        continue;
      }

      // Calculate the angular width of the shadow cast by this point
      // sin(angle_diff/2) = (robot_radius/2) / distance
      // => angle_diff/2 = asin(robot_radius / (2*distance))
      // => angle_diff = 2 * asin(robot_radius / (2 * distance)) (clip to) pi/2
      // => bucket_diff = angle_diff / angle_resolution
      double ratio = robot_radius / (2 * std::sqrt(p1_distance_sq));
      double bucket_diff = (ratio >= 1.0) ? (M_PI / 2.0) / angle_resolution : (2 * std::asin(ratio)) / angle_resolution;
      int num_buckets_covered = static_cast<int>(std::ceil(bucket_diff));

      // attempt to connect the current point to a subsequent point within the robot radius limits
      for (int j = num_buckets_covered; j > 0; j--) {
        int target_idx = (i + j) % circle_subdivisions;
        auto point2 = radial_buckets[target_idx];

        if (point2.distance_sq >= max_distance_from_seer_sqrd - epsilon) {
          continue;
        }

        auto diff_vec = (point2.point - point1.point);
        auto dist_between_points = diff_vec.norm();

        if (dist_between_points > 2 * robot_radius - epsilon) {
          continue;
        }

        auto midpoint = (point2.point + point1.point) / 2;
        double val = robot_radius * robot_radius - (dist_between_points * dist_between_points / 4.0);
        auto height_to_center = std::sqrt(std::max(0.0, val));
        auto perp_vec = Eigen::Vector2d(-diff_vec(1), diff_vec(0)).normalized();

        // determine which side of the segment faces the sensor to choose the correct circle center
        auto dot_k = midpoint.dot(perp_vec);

        Eigen::Vector2d circle_center;
        if (dot_k < 0) {
          circle_center = midpoint + height_to_center * perp_vec;
        } else {
          circle_center = midpoint - height_to_center * perp_vec;
        }

        // validate that the proposed circle is empty and no intermediate points block the sensor
        bool should_continue = false;
        for (int step = 1; step < j; step++) {
          int intermediate_idx = (i + step) % circle_subdivisions;
          auto intermediate_point = radial_buckets[intermediate_idx];

          auto p3_distance_from_seer_sq = intermediate_point.distance_sq;

          double t_interpolation = static_cast<double>(step) / j;
          Eigen::Vector2d interpolated_p = (1 - t_interpolation) * point1.point + t_interpolation * point2.point;
          if (p3_distance_from_seer_sq < interpolated_p.squaredNorm()) {
            should_continue = true;
            break;
          }

          auto vec_from_center = (intermediate_point.point - circle_center);
          auto dist_from_center = vec_from_center.norm();
          if (dist_from_center < robot_radius) {
            should_continue = true;
            break;
          }
        }

        if (should_continue) {
          continue;
        }

        // filter out the occluded points contained within the filled gap
        for (int step = 1; step < j; step++) {
          int target_idx_k = (i + step) % circle_subdivisions;
          radial_buckets[target_idx_k].metadata = -1;
        }

        // skip ahead over the newly filled virtual points to prevent recursive shadow casting
        i += j - 1;
        break;
      }
    }
  }

  void pruneRemovedBuckets(std::vector<RadialBucketPoint> &radial_buckets) const
  {
    for (int i = static_cast<int>(radial_buckets.size()) - 1; i >= 0; --i) {
      if (radial_buckets[i].metadata == -1) {
        radial_buckets.erase(radial_buckets.begin() + i);
      }
    }
  }

  void simplifyRadialBuckets(
    std::vector<RadialBucketPoint> &radial_buckets,
    double simplification_mahalanobis_error_threshold) const
  {
    // Example: simplify only obstacle points (metadata == 0) using CGAL.
    // Instead of removing simplified-out vertices from the vector, we mark
    // them with negative metadata so downstream logic can ignore them.
    if (radial_buckets.size() <= 3) {
      return;
    }

    // CGAL kernel used by the triangulation/simplification pipeline.
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using VbBase = CGAL::Polyline_simplification_2::Vertex_base_2<Kernel>;
    using Vb = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, Kernel, VbBase>;
    using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
    using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    // Constrained Delaunay triangulation is the geometric support used internally
    // by Polyline_simplification_2 to evaluate removable vertices.
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, CGAL::Exact_predicates_tag>;
    using CT = CGAL::Constrained_triangulation_plus_2<CDT>;
    namespace PS = CGAL::Polyline_simplification_2;

    constexpr int kSimplifiedOutMetadata = -3;

    auto mark_simplified_out_obstacle_run = [&](std::size_t begin_idx, std::size_t end_idx) {
      const std::size_t run_size = end_idx - begin_idx;
      if (run_size < 3) {
        return;
      }

      std::list<Kernel::Point_2> polyline;
      for (std::size_t k = begin_idx; k < end_idx; ++k) {
        const auto &point = radial_buckets[k].point;
        polyline.emplace_back(point.x(), point.y());
      }

      CT cgal_constraints;
      const bool is_closed_run = (begin_idx == 0 && end_idx == radial_buckets.size());
      const auto constraint_id = cgal_constraints.insert_constraint(polyline.begin(), polyline.end(), is_closed_run);

      std::size_t info_idx = begin_idx;
      for (auto it = cgal_constraints.vertices_in_constraint_begin(constraint_id);
           it != cgal_constraints.vertices_in_constraint_end(constraint_id);
           ++it) {
        if (info_idx < end_idx) {
          (*it)->info() = info_idx;
          ++info_idx;
        } else {
          (*it)->info() = begin_idx;
        }
      }

      const RadialBucketMahalanobisCost custom_cost(
        &radial_buckets);

      auto removed_vertices = PS::simplify(
        cgal_constraints,
        custom_cost,
        PS::Stop_above_cost_threshold(simplification_mahalanobis_error_threshold));
      (void)removed_vertices;

      std::vector<Eigen::Vector2d> simplified_points;
      simplified_points.reserve(run_size);

      for (auto it = cgal_constraints.vertices_in_constraint_begin(constraint_id);
           it != cgal_constraints.vertices_in_constraint_end(constraint_id);
           ++it) {
        const auto &cgal_point = (*it)->point();
        Eigen::Vector2d point(static_cast<double>(cgal_point.x()), static_cast<double>(cgal_point.y()));
        simplified_points.push_back(point);
      }

      if (is_closed_run && simplified_points.size() > 1) {
        const auto &first_point = simplified_points.front();
        const auto &last_point = simplified_points.back();
        if ((first_point - last_point).squaredNorm() < epsilon) {
          simplified_points.pop_back();
        }
      }

      if (simplified_points.empty()) {
        return;
      }

      for (std::size_t k = begin_idx; k < end_idx; ++k) {
        const auto &original_point = radial_buckets[k].point;
        bool kept_by_simplification = false;
        for (const auto &simplified_point : simplified_points) {
          if ((simplified_point - original_point).squaredNorm() < epsilon) {
            kept_by_simplification = true;
            break;
          }
        }

        if (!kept_by_simplification) {
          radial_buckets[k].metadata = kSimplifiedOutMetadata;
        }
      }
    };

    std::size_t idx = 0;
    while (idx < radial_buckets.size()) {
      if (radial_buckets[idx].metadata != 0) {
        ++idx;
        continue;
      }

      std::size_t run_begin = idx;
      while (idx < radial_buckets.size() && radial_buckets[idx].metadata == 0) {
        ++idx;
      }
      mark_simplified_out_obstacle_run(run_begin, idx);
    }
  }

  void publishDebugData(
    const std::string &frame_id,
    rclcpp::Time stamp,
    const std::vector<RadialBucketPoint> &radial_buckets,
    int circle_subdivisions,
    double angle_resolution,
    double max_distance,
    const std::vector<int> &publish_metadata_filter = {})
  {
    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.frame_id = frame_id;
    polygon_msg.header.stamp = stamp;

    for (const auto &bucket : radial_buckets) {
      geometry_msgs::msg::Point32 vertex;
      vertex.x = bucket.point(0);
      vertex.y = bucket.point(1);
      vertex.z = 0.0;
      polygon_msg.polygon.points.push_back(vertex);
    }
    polygon_publisher_->publish(polygon_msg);

    std::vector<RadialBucketPoint> pointcloud_buckets;
    const auto *pointcloud_buckets_to_publish = &radial_buckets;
    if (!publish_metadata_filter.empty()) {
      std::unordered_set<int> allowed_metadata(
        publish_metadata_filter.begin(),
        publish_metadata_filter.end());

      pointcloud_buckets.reserve(radial_buckets.size());
      for (const auto &bucket : radial_buckets) {
        if (allowed_metadata.find(bucket.metadata) != allowed_metadata.end()) {
          pointcloud_buckets.push_back(bucket);
        }
      }

      pointcloud_buckets_to_publish = &pointcloud_buckets;
    }

    sensor_msgs::msg::PointCloud2 pc_msg;
    pc_msg.header.frame_id = frame_id;
    pc_msg.header.stamp = stamp;

    sensor_msgs::PointCloud2Modifier modifier(pc_msg);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::INT32);
    modifier.resize(pointcloud_buckets_to_publish->size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<int32_t> iter_intensity(pc_msg, "intensity");

    for (long unsigned int i = 0; i < pointcloud_buckets_to_publish->size(); ++i) {
      auto &p = (*pointcloud_buckets_to_publish)[i].point;
      *iter_x = static_cast<float>(p(0));
      *iter_y = static_cast<float>(p(1));
      *iter_z = 0.0f;
      *iter_intensity = static_cast<int32_t>((*pointcloud_buckets_to_publish)[i].metadata);
      ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
    }
    pointcloud_publisher_->publish(pc_msg);

    visualization_msgs::msg::MarkerArray polyline_markers;
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id;
    delete_all.header.stamp = stamp;
    delete_all.ns = "debug_polylines";
    delete_all.id = 0;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    polyline_markers.markers.push_back(delete_all);

    auto is_distance_limit_metadata = [](int metadata) {
      return metadata == 1 || metadata == -2;
    };

    std::vector<geometry_msgs::msg::Point> current_polyline;
    int marker_id = 1;

    auto flush_polyline = [&]() {
      if (current_polyline.size() < 2) {
        current_polyline.clear();
        return;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = stamp;
      marker.ns = "debug_polylines";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;
      marker.color.r = 0.05f;
      marker.color.g = 0.8f;
      marker.color.b = 0.2f;
      marker.color.a = 1.0f;
      marker.points = current_polyline;
      polyline_markers.markers.push_back(marker);
      current_polyline.clear();
    };

    const std::size_t num_radial_buckets = radial_buckets.size();
    if (num_radial_buckets > 0) {
      std::size_t start_index = 0;
      bool found_distance_limit = false;
      for (std::size_t i = 0; i < num_radial_buckets; ++i) {
        if (is_distance_limit_metadata(radial_buckets[i].metadata)) {
          start_index = (i + 1) % num_radial_buckets;
          found_distance_limit = true;
          break;
        }
      }

      if (!found_distance_limit) {
        start_index = 0;
      }

      for (std::size_t offset = 0; offset < num_radial_buckets; ++offset) {
        const std::size_t idx = (start_index + offset) % num_radial_buckets;
        const auto &bucket = radial_buckets[idx];
        const int metadata = bucket.metadata;

        if (is_distance_limit_metadata(metadata)) {
          flush_polyline();
          continue;
        }

        if (metadata == 0) {
          geometry_msgs::msg::Point p;
          p.x = bucket.point(0);
          p.y = bucket.point(1);
          p.z = 0.0;
          current_polyline.push_back(p);
        }
      }

      flush_polyline();
    }

    marker_array_publisher_->publish(polyline_markers);

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

      double angle1 = radial1.angle;
      double angle2 = radial2.angle;

      // Handle wrap-around at the 2*PI boundary
      if (angle2 < angle1) {
        angle2 += 2 * M_PI;
      }

      // If the current angle surpasses angle2, advance the bucket points and try again
      if (i_angle > angle2 && i_angle < angle1 + 2 * M_PI) {
        point_idx++;
        i--;
        continue;
      }

      double p1_dist_sq = radial1.distance_sq;
      double p2_dist_sq = radial2.distance_sq;

      // check if both of the points is too close to max_distance threshold (squared!)
      double max_dist_sq = max_distance * max_distance;
      if (p1_dist_sq >= max_dist_sq - 1e-6 && p2_dist_sq >= max_dist_sq - 1e-6) {
        scan_msg.intensities[i] = point1_idx;
        continue; // leaves the default max_distance in the range
      }

      // determine where in the line segment from p1 to p2
      // the ray would cross
      // u is our directional vector (cos(theta), sin(theta))
      auto u = Eigen::Vector2d(cos(i_angle), sin(i_angle));

      // points 1 and 2
      auto p1 = radial1.point;
      auto p2 = radial2.point;

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
  }

  void publishDebugData(
    const std::string &frame_id,
    rclcpp::Time stamp,
    const std::vector<std::tuple<double, double, Eigen::Vector2d, int>> &radial_buckets,
    int circle_subdivisions,
    double angle_resolution,
    double max_distance,
    const std::vector<int> &publish_metadata_filter = {})
  {
    std::vector<RadialBucketPoint> converted_buckets;
    converted_buckets.reserve(radial_buckets.size());

    for (const auto &bucket : radial_buckets) {
      RadialBucketPoint converted_bucket;
      converted_bucket.angle = std::get<0>(bucket);
      converted_bucket.distance_sq = std::get<1>(bucket);
      converted_bucket.point = std::get<2>(bucket);
      converted_bucket.metadata = std::get<3>(bucket);
      converted_bucket.dbscan_label = -1;
      converted_buckets.push_back(converted_bucket);
    }

    publishDebugData(
      frame_id,
      stamp,
      converted_buckets,
      circle_subdivisions,
      angle_resolution,
      max_distance,
      publish_metadata_filter);
  }

  void onInitialize(grid_map::GridMap &grid_map) override
  {
    node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
    node_->declare_parameter(name_ + ".invert_selection", false);
    node_->declare_parameter(name_ + ".output_layer", "distance_field");
    node_->declare_parameter(name_ + ".circle_subdivisions", 360);
    node_->declare_parameter(name_ + ".robot_radius", 0.5);
    node_->declare_parameter(name_ + ".max_distance", 5.0);
    node_->declare_parameter(name_ + ".slice_dbscan_eps", 0.5);
    node_->declare_parameter(name_ + ".slice_dbscan_min_points", 10);
    // Squared geometric error threshold used by CGAL simplification stop criterion.
    node_->declare_parameter(name_ + ".simplification_mahalanobis_error_threshold", 0.0025);
    // Draw covariance markers at z=0 (false) or at the actual centroid height (true).
    node_->declare_parameter(name_ + ".covariance_markers_use_actual_z", false);
    // If empty, all metadata values are published in debug outputs.
    node_->declare_parameter(name_ + ".debug_publish_metadata_filter", std::vector<int64_t>());

    grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);

    polygon_publisher_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("~/debug_polygon", 10);
    process_time_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("~/" + name_ + "/process_time", 10);
    pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug_pointcloud", 10);
    marker_array_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug_polylines", 10);
    laserscan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("~/debug_laserscan", 10);
    pointcloud_multiarray_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug_pointcloud_multiarray", 10);
    distances_multiarray_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("~/debug_distances_multiarray", 10);
  }

  void onProcess(PartitioningContext &context, grid_map::GridMap &grid_map) override
  {
    auto tstart = std::chrono::high_resolution_clock::now();

    auto params = loadProcessParams();

    grid_map[params.output_layer].setConstant(std::numeric_limits<float>::quiet_NaN());

    std::shared_ptr<const open3d::geometry::PointCloud> o3d_pc;
    try
    {
      o3d_pc = sfframework::SFGeneratorUtils::cloud_from_tags(
        context,
        params.input_tags,
        params.invert_selection
      );
    }
    catch (const std::out_of_range &e)
    {
      RCLCPP_WARN(node_->get_logger(),
        "SF strategy plugin [%s] tried to find a list of clusters at a tag but it is missing. Skipping...",
        name_.c_str());
      return;
    }

    auto point_polar_data = buildPolarData(*o3d_pc);

    double max_distance_from_seer_sqrd = params.max_distance_from_seer * params.max_distance_from_seer;

    double angle_resolution = 2 * M_PI / params.circle_subdivisions;

    auto radial_buckets_multiarray = bucketizePolarData(
      point_polar_data,
      params.circle_subdivisions,
      angle_resolution);

    runDbscanOnBuckets(
      radial_buckets_multiarray,
      params.slice_dbscan_eps,
      params.slice_dbscan_min_points);

    // For each angular slice (bucket) compute per-cluster centroids and
    // select the cluster whose centroid (XY) is closest to the sensor origin (0,0).
    std::vector<int> selected_label_per_bucket(params.circle_subdivisions, -1);
    std::vector<Eigen::Vector3d> selected_centroid_per_bucket(
      params.circle_subdivisions,
      Eigen::Vector3d::Zero());
    std::vector<Eigen::Matrix3d> selected_covariance_per_bucket(
      params.circle_subdivisions,
      Eigen::Matrix3d::Zero());

    selectClosestClusters(
      radial_buckets_multiarray,
      selected_label_per_bucket,
      selected_centroid_per_bucket,
      selected_covariance_per_bucket);

    auto radial_buckets = buildRadialBuckets(
      params.circle_subdivisions,
      angle_resolution,
      params.max_distance_from_seer,
      max_distance_from_seer_sqrd,
      radial_buckets_multiarray,
      selected_label_per_bucket,
      selected_centroid_per_bucket,
      selected_covariance_per_bucket);

    publishPointcloudMultiarray(
      grid_map,
      context,
      params.circle_subdivisions,
      radial_buckets_multiarray,
      selected_label_per_bucket);

    publishCovarianceMarkers(
      grid_map,
      context,
      radial_buckets,
      params.covariance_markers_use_actual_z);

    fillOcclusionsBPA(
      radial_buckets,
      params.circle_subdivisions,
      angle_resolution,
      max_distance_from_seer_sqrd,
      params.robot_radius);

    pruneRemovedBuckets(radial_buckets);

    simplifyRadialBuckets(
      radial_buckets,
      params.simplification_mahalanobis_error_threshold);

    publishDebugData(
      grid_map.getFrameId(),
      rclcpp::Time(context.header.stamp),
      radial_buckets,
      params.circle_subdivisions,
      angle_resolution,
      params.max_distance_from_seer,
      params.publish_metadata_filter);

    auto tend = std::chrono::high_resolution_clock::now();
    auto duration = tend - tstart;
    std_msgs::msg::Float64 time_msg;
    time_msg.data = std::chrono::duration<double, std::milli>(duration).count();
    process_time_publisher_->publish(time_msg);
  }
};
} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::CompressedStochasticPolygon, sfframework::SFStrategy)
