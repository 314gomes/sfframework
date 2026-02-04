#ifndef SFFRAMEWORK_SFGENERATOR_UTILS_HPP_
#define SFFRAMEWORK_SFGENERATOR_UTILS_HPP_

#include <memory>
#include <string>
#include <vector>
#include <open3d/Open3D.h>
#include "sfframework/partitioning_context.hpp"

namespace sfframework
{

class SFGeneratorUtils
{
public:
  // Delete constructor to prevent instantiation
  SFGeneratorUtils() = delete;

  /// @brief Extracts a sub-cloud based on semantic tags.
  /// @param context Context with source cloud and clusters.
  /// @param tags Tags to select.
  /// @param invert_selection True to exclude the specified tags instead of including them.
  /// @param indices_map_ Output vector of selected original indices.
  /// @return The resulting selected point cloud.
  static std::shared_ptr<const open3d::geometry::PointCloud> cloud_from_tags(const PartitioningContext & context, const std::vector<std::string> & tags, const bool invert_selection, std::vector<size_t> &indices_map_);

  /// @brief Extracts a sub-cloud based on semantic tags (optimized, no index map return).
  /// @param context Context with source cloud and clusters.
  /// @param tags Tags to select.
  /// @param invert_selection True to exclude the specified tags instead of including them.
  /// @return The resulting selected point cloud.
  static std::shared_ptr<const open3d::geometry::PointCloud> cloud_from_tags(const PartitioningContext & context, const std::vector<std::string> & tags, const bool invert_selection);
};

}  // namespace sfframework

#endif  // SFFRAMEWORK_SFGENERATOR_UTILS_HPP_
