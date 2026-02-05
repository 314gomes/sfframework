#include <pluginlib/class_list_macros.hpp>
#include "sfframework/sf_strategy.hpp"
#include "sfframework/sfgenerator_utils.hpp"
#include <omp.h>

namespace sfframework
{
  class GaussianRepulsion : public SFStrategy
  {
    void onInitialize(grid_map::GridMap &grid_map) override
    {
      node_->declare_parameter(name_ + ".A", 1.0);
      node_->declare_parameter(name_ + ".sigma", 0.5);
      node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
      node_->declare_parameter(name_ + ".invert_selection", false);
      node_->declare_parameter(name_ + ".output_layer", "gaussian_repulsion");
      grid_map.add(node_->get_parameter(name_ + ".output_layer").as_string(), 0.0);
    }

    void onProcess(PartitioningContext &context, grid_map::GridMap &grid_map) override
    {
      std::shared_ptr<const open3d::geometry::PointCloud> o3d_pc;

      // attempt to get point cloud from context
      try
      {
        o3d_pc = sfframework::SFGeneratorUtils::cloud_from_tags(
          context,
          node_->get_parameter(name_ + ".input_tags").as_string_array(),
          node_->get_parameter(name_ + ".invert_selection").as_bool()
        );
      }
      catch (const std::out_of_range &e)
      {
        RCLCPP_WARN(node_->get_logger(), "SF strategy plugin [%s] tried to find a list of clusters at a tag but it is missing. Skipping...", name_.c_str());
        return;
      }

      // Reset the layer to zero before accumulating new values
      grid_map[this->node_->get_parameter(name_ + ".output_layer").as_string()].setConstant(0.0);

      // Distance above which the potential is negligible and we can skip computation for efficiency
      auto sigma = this->node_->get_parameter(name_ + ".sigma").as_double();
      auto distance_threshold_sqrd = sigma * sigma * 9.0;
      auto A = this->node_->get_parameter(name_ + ".A").as_double();
      auto two_sigma_sqrd = 2.0 * sigma * sigma;
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();

      // Compute potential field using omp
      #pragma omp parallel for
      for (size_t i = 0; i < o3d_pc->points_.size(); ++i)
      {
        auto point = o3d_pc->points_[i];
        auto point_position_on_grid = grid_map::Position(point(0), point(1));

        // iterate over every every gridmap cell
        for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
        {
          grid_map::Position cell_position(0, 0);
          grid_map.getPosition(*iterator, cell_position);
          double distance_sqrd = (point_position_on_grid - cell_position).squaredNorm();

          if (distance_sqrd < distance_threshold_sqrd)
          {
            // Potential field decreases with distance_sqrd
            double potential = A * std::exp(-distance_sqrd / two_sigma_sqrd);
            if (grid_map.at(output_layer, *iterator) < potential)
            {
              grid_map.at(output_layer, *iterator) = potential;
            }
          }
        }
      }
    }
  };

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
      auto zeta = this->node_->get_parameter(name_ + ".zeta").as_double();
      auto center_x = this->node_->get_parameter(name_ + ".center_x").as_double();
      auto center_y = this->node_->get_parameter(name_ + ".center_y").as_double();
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();

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
      auto input_layers = this->node_->get_parameter(name_ + ".input_layers").as_string_array();
      auto weights = this->node_->get_parameter(name_ + ".weights").as_double_array();
      auto output_layer = this->node_->get_parameter(name_ + ".output_layer").as_string();

      if (input_layers.size() != weights.size())
      {
        RCLCPP_ERROR(node_->get_logger(), "SF strategy plugin [%s] has a different number of input layers and weights. Skipping...", name_.c_str());
        return;
      }

      // Check if every input layer exists
      for (size_t i = 0; i < input_layers.size(); ++i){
        if (!grid_map.exists(input_layers[i]))
        {
          RCLCPP_WARN(node_->get_logger(), "SF strategy plugin [%s] tried to find an input layer that does not exist. Skipping...", name_.c_str());
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

PLUGINLIB_EXPORT_CLASS(sfframework::GaussianRepulsion, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::ParabolicRest, sfframework::SFStrategy)
PLUGINLIB_EXPORT_CLASS(sfframework::WeightedSummation, sfframework::SFStrategy)