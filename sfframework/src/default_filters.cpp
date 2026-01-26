#include "sfframework/filter_base.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace sfframework
{

class StatisticalOutlierRemovalFilter : public FilterBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node) override
  {
    name_ = name;
    node_ = node;
    // Default values matching the original code
    node_->declare_parameter(name_ + ".nb_neighbors", 30);
    node_->declare_parameter(name_ + ".std_ratio", 2.0);
  }

  std::shared_ptr<open3d::geometry::PointCloud> filter(
    const std::shared_ptr<open3d::geometry::PointCloud> & input) override
  {
    int nb_neighbors = node_->get_parameter(name_ + ".nb_neighbors").as_int();
    double std_ratio = node_->get_parameter(name_ + ".std_ratio").as_double();

    std::shared_ptr<open3d::geometry::PointCloud> output;
    std::tie(output, std::ignore) = input->RemoveStatisticalOutliers(nb_neighbors, std_ratio);
    return output;
  }

private:
  std::string name_;
  rclcpp::Node * node_;
};

class VoxelDownSampleFilter : public FilterBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node) override
  {
    name_ = name;
    node_ = node;
    // Default value matching original code
    node_->declare_parameter(name_ + ".voxel_size", 0.2);
  }

  std::shared_ptr<open3d::geometry::PointCloud> filter(
    const std::shared_ptr<open3d::geometry::PointCloud> & input) override
  {
    double voxel_size = node_->get_parameter(name_ + ".voxel_size").as_double();
    return input->VoxelDownSample(voxel_size);
  }

private:
  std::string name_;
  rclcpp::Node * node_;
};

}  // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::StatisticalOutlierRemovalFilter, sfframework::FilterBase)
PLUGINLIB_EXPORT_CLASS(sfframework::VoxelDownSampleFilter, sfframework::FilterBase)
