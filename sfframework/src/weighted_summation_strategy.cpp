#include <pluginlib/class_list_macros.hpp>

#include "sfframework/sf_strategy.hpp"

#include <vector>

namespace sfframework
{
class WeightedSummation : public SFStrategy
{
  void onInitialize(grid_map::GridMap &grid_map) override
  {
    node_->declare_parameter(name_ + ".input_layers", std::vector<std::string>());
    node_->declare_parameter(name_ + ".weights", std::vector<double>());
    node_->declare_parameter(name_ + ".output_layer", "potential");

    grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
  }

  void onProcess(PartitioningContext & /*context*/, grid_map::GridMap &grid_map) override
  {
    auto input_layers = node_->get_parameter(name_ + ".input_layers").as_string_array();
    auto weights = node_->get_parameter(name_ + ".weights").as_double_array();
    auto output_layer = node_->get_parameter(name_ + ".output_layer").as_string();

    if (input_layers.size() != weights.size())
    {
      RCLCPP_ERROR(node_->get_logger(),
        "SF strategy plugin [%s] has a different number of input layers and weights. Skipping...",
        name_.c_str());
      return;
    }

    // Check if every input layer exists
    for (size_t i = 0; i < input_layers.size(); ++i) {
      if (!grid_map.exists(input_layers[i]))
      {
        RCLCPP_WARN(node_->get_logger(),
          "SF strategy plugin [%s] tried to find an input layer that does not exist. Skipping...",
          name_.c_str());
        return;
      }
    }

    // Compute weighted summation
    grid_map[output_layer].setConstant(0.0);
    for (size_t i = 0; i < input_layers.size(); ++i)
    {
      grid_map[output_layer] += weights[i] * grid_map[input_layers[i]];
    }
  }
};
} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::WeightedSummation, sfframework::SFStrategy)
