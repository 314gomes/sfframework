#include "sfframework/partitioning_strategy.hpp"
#include "sfframework/cluster_types.hpp"
#include "sfframework/exceptions.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <omp.h>

namespace sfframework
{

	class TrivialPlaneSegmentation : public PartitioningStrategy
	{
	public:
		void onInitialize() override
		{
			this->node_->declare_parameter(name_ + ".plane_name", "ground");
			rcl_interfaces::msg::ParameterDescriptor plane_link_desc;
			plane_link_desc.description = "a link touching the plane and parallel to it, usually odom or base_footprint)";
			this->node_->declare_parameter(name_ + ".plane_link", "robot_base_footprint", plane_link_desc);
			this->node_->declare_parameter(name_ + ".distance_threshold", 0.1);
		}
		void onProcess(PartitioningContext &context) override
		{
			std::string plane_tag = node_->get_parameter(name_ + ".plane_name").as_string();
			std::string plane_link = node_->get_parameter(name_ + ".plane_link").as_string();
			double distance_threshold = node_->get_parameter(name_ + ".distance_threshold").as_double();
			
			// Get plane equation in relation to the cloud frame using TF
			geometry_msgs::msg::TransformStamped transform;
			try {
				transform = this->tf_buffer_->lookupTransform(
					context.header.frame_id,
					plane_link,
					rclcpp::Time(context.header.stamp)
				);
			} catch (const tf2::TransformException & ex) {
				RCLCPP_WARN(node_->get_logger(), "TrivialPlaneSegmentation: %s", ex.what());
				// throw skip frame exception
				throw SkipFrameException("Could not get transform to plane link " + plane_link);
			}

			// Plane in plane_link is z=0. Normal (0,0,1), Point (0,0,0).
			// Transform to cloud frame.
			Eigen::Quaterniond q(
				transform.transform.rotation.w,
				transform.transform.rotation.x,
				transform.transform.rotation.y,
				transform.transform.rotation.z
			);
			Eigen::Vector3d n = q * Eigen::Vector3d::UnitZ();
			Eigen::Vector3d p0(
				transform.transform.translation.x,
				transform.transform.translation.y,
				transform.transform.translation.z
			);

			// Plane equation: n.x + d = 0 -> n.(p - p0) = 0 -> n.p - n.p0 = 0
			double d = -n.dot(p0);
			Eigen::Vector4d coefficients(n.x(), n.y(), n.z(), d);

			PartitioningCluster plane_cluster;
			plane_cluster.attributes["model"] = PlaneModel{coefficients};

			// remove points whose z value is bellow the plane (with some threshold)
			#pragma omp parallel
			{
				std::vector<size_t> local_indices;
				#pragma omp for nowait
				for (size_t i = 0; i < input_cloud_->points_.size(); ++i) {
					const auto& pt = input_cloud_->points_[i];
					double plane_z = -(coefficients(0) * pt.x() + coefficients(1) * pt.y() + coefficients(3)) / coefficients(2);
					if (pt.z() < plane_z + distance_threshold) {
						local_indices.push_back(mapToOriginalIndex(i));
					}
				}
				#pragma omp critical
				plane_cluster.indices.insert(plane_cluster.indices.end(), local_indices.begin(), local_indices.end());
			}

			context.clusters_registry[plane_tag].push_back(plane_cluster);
		}
	};

	class RansacSegmentation : public PartitioningStrategy
	{

	public:
		void onInitialize() override
		{
			// declare parameters used in SegmentPlane
			node_->declare_parameter(name_ + ".distance_threshold", 0.4);
			node_->declare_parameter(name_ + ".ransac_n", 3);
			node_->declare_parameter(name_ + ".num_iterations", 1000);
			node_->declare_parameter(name_ + ".probability", 0.99999999);

			node_->declare_parameter(name_ + ".plane_name", "ground");
			node_->declare_parameter(name_ + ".non_plane_name", "non-" + node_->get_parameter(name_ + ".plane_name").as_string());

			node_->declare_parameter(name_ + ".max_inclination_rad", 0.785398163);
		}
		void onProcess(PartitioningContext &context){
			int ransac_n = node_->get_parameter(name_ + ".ransac_n").as_int();
			if (input_cloud_->points_.size() < static_cast<size_t>(ransac_n)) {
				// warn and skip
				RCLCPP_WARN(node_->get_logger(), "Partitioning plugin [%s] found input cloud size (%zu) smaller than ransac_n (%d). Skipping plane segmentation.", name_.c_str(), input_cloud_->points_.size(), ransac_n);
				return;
			}

			auto [coefficients, inliers] = input_cloud_->SegmentPlane(
				node_->get_parameter(name_ + ".distance_threshold").as_double(),
				ransac_n,
				node_->get_parameter(name_ + ".num_iterations").as_int(),
				node_->get_parameter(name_ + ".probability").as_double()
			);

			if (inliers.empty()) {
				return;
			}

			// Check inclination: angle between plane normal (a,b,c) and vertical (0,0,1)
			// cos(theta) = c (assuming normalized coefficients)
			double inclination = std::acos(std::abs(coefficients(2)));
			double max_inclination = node_->get_parameter(name_ + ".max_inclination_rad").as_double();
			if (inclination > max_inclination) {
				throw SkipFrameException("Plane inclination " + std::to_string(inclination) + " exceeds limit " + std::to_string(max_inclination));
			}

			PartitioningCluster plane_cluster;
			if (!needsMapping()) {
				plane_cluster.indices = inliers;
			} else {
				plane_cluster.indices.reserve(inliers.size());
				for (size_t idx : inliers) plane_cluster.indices.push_back(mapToOriginalIndex(idx));
			}

			plane_cluster.attributes["model"] = PlaneModel{coefficients};

			std::string plane_tag = node_->get_parameter(name_ + ".plane_name").as_string();
			context.clusters_registry[plane_tag].push_back(plane_cluster);

			// Create a boolean mask to efficiently identify points not in the plane
			size_t num_points = input_cloud_->points_.size();
			std::vector<bool> is_inlier(num_points, false);
			for (size_t idx : inliers) {
				is_inlier[idx] = true;
			}

			PartitioningCluster non_plane_cluster;
			non_plane_cluster.indices.reserve(num_points - inliers.size());
			for (size_t i = 0; i < num_points; ++i) {
				if (!is_inlier[i]) {
					non_plane_cluster.indices.push_back(mapToOriginalIndex(i));
				}
			}

			std::string non_plane_tag = node_->get_parameter(name_ + ".non_plane_name").as_string();
			context.clusters_registry[non_plane_tag].push_back(non_plane_cluster);
		}
	};
	class DBSCAN : public PartitioningStrategy
	{

	public:
		void onInitialize() override
		{
			// declare parameters used
			node_->declare_parameter(name_ + ".eps", 0.5);
			node_->declare_parameter(name_ + ".min_points", 10);

			node_->declare_parameter(name_ + ".noise_tag", "noise");
			node_->declare_parameter(name_ + ".clusters_tag", "clusters");
		}
		void onProcess(PartitioningContext &context){

			auto noise_tag = node_->get_parameter(name_ + ".noise_tag").as_string();
			auto clusters_tag = node_->get_parameter(name_ + ".clusters_tag").as_string();

			auto point_labels = this->input_cloud_->ClusterDBSCAN(
				node_->get_parameter(name_ + ".eps").as_double(),
				node_->get_parameter(name_ + ".min_points").as_int()
			);
			
			// Clear previous results from this partitioner
			context.clusters_registry[clusters_tag].clear();
			context.clusters_registry[noise_tag].clear();

			// Find max label to pre-allocate vectors in the registry
			int max_label = -1;
			for (int label : point_labels) {
				if (label > max_label) max_label = label;
			}

			// Resize the registry vector to hold all clusters directly
			auto& cluster_list = context.clusters_registry[clusters_tag];
			cluster_list.resize(max_label + 1);

			std::vector<size_t> noise_indices;

			// Group original indices directly into the registry
			for (size_t i = 0; i < point_labels.size(); ++i) {
				int label = point_labels[i];
				size_t original_index = mapToOriginalIndex(i);

				if (label == -1) {
					noise_indices.push_back(original_index);
				} else {
					cluster_list[label].indices.push_back(original_index);
				}
			}

			if (!noise_indices.empty()) {
				PartitioningCluster noise_cluster;
				noise_cluster.indices = std::move(noise_indices);
				context.clusters_registry[noise_tag].push_back(std::move(noise_cluster));
			}
		}
	};

} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::TrivialPlaneSegmentation, sfframework::PartitioningStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::RansacSegmentation, sfframework::PartitioningStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::DBSCAN, sfframework::PartitioningStrategy)
