#include "sfframework/sfgenerator_utils.hpp"

std::shared_ptr<const open3d::geometry::PointCloud> sfframework::SFGeneratorUtils::cloud_from_tags(const PartitioningContext &context, const std::vector<std::string> &tags, const bool invert_selection, std::vector<size_t> &indices_map_)
{
  std::shared_ptr<const open3d::geometry::PointCloud> output_cloud;

  if (tags.empty())
  {
    output_cloud = context.cloud;
  }
  else
  {
    std::vector<size_t> input_indices_;
    for (const auto &tag : tags)
    {
      std::vector<PartitioningCluster> clusters_at_tag;

      clusters_at_tag = context.clusters_registry.at(tag);
      for (const auto &cluster : clusters_at_tag)
      {
        input_indices_.insert(input_indices_.end(), cluster.indices.begin(), cluster.indices.end());
      }
    }

    if (invert_selection)
    {
      indices_map_.clear();
      size_t num_points = context.cloud->points_.size();
      std::vector<bool> mask(num_points, true);
      for (size_t idx : input_indices_)
      {
        if (idx < num_points)
          mask[idx] = false;
      }
      for (size_t i = 0; i < num_points; ++i)
      {
        if (mask[i])
          indices_map_.push_back(i);
      }
    }
    else
    {
      indices_map_ = input_indices_;
    }
    output_cloud = context.cloud->SelectByIndex(indices_map_, false);
  }
  return output_cloud;
}

std::shared_ptr<const open3d::geometry::PointCloud> sfframework::SFGeneratorUtils::cloud_from_tags(const PartitioningContext &context, const std::vector<std::string> &tags, const bool invert_selection)
{
  if (tags.empty())
  {
    return context.cloud;
  }

  std::vector<size_t> input_indices_;
  for (const auto &tag : tags)
  {
    const auto &clusters_at_tag = context.clusters_registry.at(tag);
    for (const auto &cluster : clusters_at_tag)
    {
      input_indices_.insert(input_indices_.end(), cluster.indices.begin(), cluster.indices.end());
    }
  }

  return context.cloud->SelectByIndex(input_indices_, invert_selection);
}