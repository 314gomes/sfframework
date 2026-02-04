#include "sfframework/partitioning_strategy.hpp"
#include "sfframework/cluster_types.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace sfframework
{
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

PLUGINLIB_EXPORT_CLASS(sfframework::RansacSegmentation, sfframework::PartitioningStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::DBSCAN, sfframework::PartitioningStrategy)
