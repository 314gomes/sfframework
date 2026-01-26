#ifndef SFFRAMEWORK_FILTER_BASE_HPP_
#define SFFRAMEWORK_FILTER_BASE_HPP_

#include <memory>
#include <string>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>

namespace sfframework
{
class FilterBase
{
public:
  using Ptr = std::shared_ptr<FilterBase>;

  virtual ~FilterBase() = default;

  virtual void initialize(const std::string & name, rclcpp::Node * node) = 0;

  virtual std::shared_ptr<open3d::geometry::PointCloud> filter(
    const std::shared_ptr<open3d::geometry::PointCloud> & input) = 0;
};
}  // namespace sfframework

#endif  // SFFRAMEWORK_FILTER_BASE_HPP_
