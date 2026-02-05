#include <pluginlib/class_list_macros.hpp>
#include "sfframework/sf_strategy.hpp"
#include "sfframework/sfgenerator_utils.hpp"
#include <omp.h>

namespace sfframework
{
  class GaussianRepulsion : public SFStrategy
  {
    void onInitialize() override
    {
      node_->declare_parameter(name_ + ".A", 0.001);
      node_->declare_parameter(name_ + ".sigma", 0.5);
      node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
      node_->declare_parameter(name_ + ".invert_selection", false);
      node_->declare_parameter(name_ + ".output_layer", "gaussian_repulsion");
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

      // Distance above which the potential is negligible and we can skip computation for efficiency
      auto distance_threshold = this->node_->get_parameter("sigma").as_double() * 3.0;
      auto A = this->node_->get_parameter(name_ + ".A").as_double();
      auto two_sigma_sqrd = this->node_->get_parameter(name_ + ".sigma").as_double();
      two_sigma_sqrd = two_sigma_sqrd * two_sigma_sqrd;
      two_sigma_sqrd = two_sigma_sqrd * 2.0;

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

          if (distance_sqrd < distance_threshold)
          {
            // Potential field decreases with distance_sqrd (e.g., Gaussian)
            double potential = A * std::exp(-distance_sqrd / two_sigma_sqrd);
            if (grid_map.at("potential", *iterator) < potential)
            {
              grid_map.at("potential", *iterator) = potential;
            }
          }
        }
      }
    }
  };

} // namespace sfframework

PLUGINLIB_EXPORT_CLASS(sfframework::GaussianRepulsion, sfframework::SFStrategy)
