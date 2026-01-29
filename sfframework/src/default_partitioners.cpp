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
		void process(PartitioningContext &context){
			auto [coefficients, inliers] = context.cloud->SegmentPlane(
				node_->get_parameter(name_ + ".distance_threshold").as_double(),
				node_->get_parameter(name_ + ".ransac_n").as_int(),
				node_->get_parameter(name_ + ".num_iterations").as_int(),
				node_->get_parameter(name_ + ".probability").as_double()
			);

			if (inliers.empty()) {
				return;
			}

			PartitioningCluster plane_cluster;
			plane_cluster.indices = inliers;
			plane_cluster.attributes["model"] = PlaneModel{coefficients};

			std::string plane_tag = node_->get_parameter(name_ + ".plane_name").as_string();
			context.clusters_registry[plane_tag].push_back(plane_cluster);

			// Create a boolean mask to efficiently identify points not in the plane
			size_t num_points = context.cloud->points_.size();
			std::vector<bool> is_inlier(num_points, false);
			for (size_t idx : inliers) {
				is_inlier[idx] = true;
			}

			PartitioningCluster non_plane_cluster;
			non_plane_cluster.indices.reserve(num_points - inliers.size());
			for (size_t i = 0; i < num_points; ++i) {
				if (!is_inlier[i]) {
					non_plane_cluster.indices.push_back(i);
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
		void process(PartitioningContext &context){
			if (context.clusters_registry.find("non-ground") == context.clusters_registry.end() || context.clusters_registry.at("non-ground").empty())
			{
				RCLCPP_WARN(node_->get_logger(), "DBSCAN: 'non-ground' cluster not found or empty, skipping partitioner.");
				return;
			}

			const auto& non_ground_indices = context.clusters_registry.at("non-ground")[0].indices;
			auto non_ground_cloud = context.cloud->SelectByIndex(non_ground_indices);

			auto noise_tag = node_->get_parameter(name_ + ".noise_tag").as_string();
			auto clusters_tag = node_->get_parameter(name_ + ".clusters_tag").as_string();

			auto point_labels = non_ground_cloud->ClusterDBSCAN(
				node_->get_parameter(name_ + ".eps").as_double(),
				node_->get_parameter(name_ + ".min_points").as_int()
			);
			
			// Group original indices by cluster label
			std::map<int, std::vector<size_t>> clusters;
			for (size_t i = 0; i < point_labels.size(); ++i) {
				// Map back to original index
				clusters[point_labels[i]].push_back(non_ground_indices[i]);
			}

			// Clear previous results from this partitioner
			context.clusters_registry[clusters_tag].clear();
			context.clusters_registry[noise_tag].clear();

			// Add new clusters to registry
			for(auto const& [label, indices] : clusters)
			{
				if (label == -1) { // Noise points
					if (!indices.empty()) {
						PartitioningCluster noise_cluster;
						noise_cluster.indices = indices;
						context.clusters_registry[noise_tag].push_back(noise_cluster);
					}
				} else { // Regular clusters
					if (!indices.empty()) {
						PartitioningCluster cluster;
						cluster.indices = indices;
						context.clusters_registry[clusters_tag].push_back(cluster);
					}
				}
			}
		}
	};

} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::RansacSegmentation, sfframework::PartitioningStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::DBSCAN, sfframework::PartitioningStrategy)
