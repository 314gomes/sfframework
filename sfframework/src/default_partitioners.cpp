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
			node_->declare_parameter(name_ + ".non_plane_name", "obstacles");

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
} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::RansacSegmentation, sfframework::PartitioningStrategy)
