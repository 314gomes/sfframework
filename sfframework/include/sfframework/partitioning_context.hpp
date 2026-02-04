#ifndef SFFRAMEWORK_PARTITIONING_CONTEXT_HPP_
#define SFFRAMEWORK_PARTITIONING_CONTEXT_HPP_

#include <memory>
#include <open3d/Open3D.h>
#include <sfframework/partitioning_cluster.hpp>

struct PartitioningContext {
    // Read-Only pointer to the raw sensor data
    std::shared_ptr<const open3d::geometry::PointCloud> cloud;

    // The Registry: Maps a semantic Tag to a list of Clusters
    // e.g., "ground" -> [Cluster], "obstacles" -> [Cluster, Cluster...]
    std::map<std::string, std::vector<PartitioningCluster>> clusters_registry;
};

#endif  // SFFRAMEWORK_PARTITIONING_CONTEXT_HPP_