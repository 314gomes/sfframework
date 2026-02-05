#ifndef SF_STRATEGY_HPP_
#define SF_STRATEGY_HPP_

#include <memory>
#include <string>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>
#include "sfframework/partitioning_context.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

namespace sfframework
{
  class SFStrategy
  {
  public:
    using Ptr = std::shared_ptr<SFStrategy>;

    virtual ~SFStrategy() = default;

    void initialize(rclcpp::Node *node, const std::string &name)
    {
      node_ = node;
      name_ = name;

      this->onInitialize();
    };

    void process(PartitioningContext &context, grid_map::GridMap &grid_map)
    {
      this->onProcess(context, grid_map);
    }

  protected:
    // The logic step: Read input_cloud_ -> Process -> Write to Context
    virtual void onProcess(PartitioningContext &context, grid_map::GridMap &grid_map) = 0;
    virtual void onInitialize() = 0;

    std::string name_;
    rclcpp::Node *node_;

  };
} // namespace sfframework

#endif // SFFRAMEWORK_PARTITIONING_STRATEGY_HPP_
