#include <pluginlib/class_list_macros.hpp>

#include "sfframework/sf_strategy.hpp"

#include <vector>

namespace sfframework
{
// Resting attractive potential centered at a user-defined point
class ParabolicRest : public SFStrategy
{
  void onInitialize(grid_map::GridMap &grid_map) override
  {
    node_->declare_parameter(name_ + ".zeta", 0.1);
    node_->declare_parameter(name_ + ".center_x", 0.0);
    node_->declare_parameter(name_ + ".center_y", 0.0);
    node_->declare_parameter(name_ + ".output_layer", "parabolic_rest");
    grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
  }

  void onProcess(PartitioningContext & /*context*/, grid_map::GridMap &grid_map) override
  {
    auto zeta = node_->get_parameter(name_ + ".zeta").as_double();
    auto center_x = node_->get_parameter(name_ + ".center_x").as_double();
    auto center_y = node_->get_parameter(name_ + ".center_y").as_double();
    auto output_layer = node_->get_parameter(name_ + ".output_layer").as_string();

    // Compute potential field using omp
    // #pragma omp parallel for
    for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      grid_map::Position cell_position(0, 0);
      grid_map.getPosition(*iterator, cell_position);
      double distance_sqrd = (cell_position(0) - center_x) * (cell_position(0) - center_x) +
                            (cell_position(1) - center_y) * (cell_position(1) - center_y);

      // Parabolic potential field increases with distance_sqrd
      double potential = zeta * distance_sqrd;
      grid_map.at(output_layer, *iterator) = potential;
    }
  }
};
} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::ParabolicRest, sfframework::SFStrategy)
