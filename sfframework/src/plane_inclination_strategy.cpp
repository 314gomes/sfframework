#include <pluginlib/class_list_macros.hpp>

#include "sfframework/sf_strategy.hpp"

#include "sfframework/cluster_types.hpp"

#include <any>

namespace sfframework
{
class PlaneInclination : public SFStrategy
{
  void onInitialize(grid_map::GridMap &grid_map) override
  {
    node_->declare_parameter(name_ + ".input_tag", "ground");

    node_->declare_parameter(name_ + ".output_layer", "plane_inclination");
    grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
  }

  void onProcess(PartitioningContext &context, grid_map::GridMap &grid_map) override
  {
    auto input_tag = node_->get_parameter(name_ + ".input_tag").as_string();
    auto output_layer = node_->get_parameter(name_ + ".output_layer").as_string();
    // minimum value for c to avoid division by zero resulting in overly inclined plane
    double threshold = 0.01;

    PlaneModel plane;
    try
    {
      // plane = (PlaneModel) context->clusters_registry.at(input_tag).at(0).attributes.at("model");
      plane = std::any_cast<PlaneModel>(context.clusters_registry.at(input_tag).at(0).attributes.at("model"));
    }
    catch (const std::out_of_range &e)
    {
      RCLCPP_WARN(node_->get_logger(),
        "SF strategy plugin [%s] tried to find a plane model in the cluster attributes but it is missing. Skipping...",
        name_.c_str());
      return;
    }

    auto a = plane.coefficients(0);
    auto b = plane.coefficients(1);
    auto c = plane.coefficients(2);
    auto d = plane.coefficients(3);

    // print plane model for debugging
    // RCLCPP_INFO(node_->get_logger(), "Plane model: a=%f, b=%f, c=%f, d=%f", a, b, c, d);

    if (c <= threshold)
    {
      RCLCPP_WARN(node_->get_logger(),
        "SF strategy plugin [%s] found a plane model with c too close to 0.",
        name_.c_str());
      grid_map[output_layer].setConstant(0.0);
      return;
    }

    // Compute inclination field
    for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      grid_map::Position cell_position(0, 0);
      grid_map.getPosition(*iterator, cell_position);
      double x = cell_position(0);
      double y = cell_position(1);
      // Compute z from plane equation: ax + by + cz + d = 0  =>  z = -(a*x + b*y + d) / c
      double z = (- (a * x + b * y + d) / c);
      grid_map.at(output_layer, *iterator) = z;
    }
  }
};
} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::PlaneInclination, sfframework::SFStrategy)
