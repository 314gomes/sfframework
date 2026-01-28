#ifndef SFFRAMEWORK_PARTITIONING_STRATEGY_HPP_
#define SFFRAMEWORK_PARTITIONING_STRATEGY_HPP_

#include <memory>
#include <string>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>
#include "sfframework/partitioning_context.hpp"

namespace sfframework
{
  class PartitioningStrategy
  {
  public:
    using Ptr = std::shared_ptr<PartitioningStrategy>;

    virtual ~PartitioningStrategy() = default;

    void initialize(rclcpp::Node *node, const std::string &name)
    {
      node_ = node;
      name_ = name;
      onInitialize();
    };

    // The logic step: Read from Context -> Process -> Write to Context
    virtual void process(PartitioningContext &context) = 0;

  protected:
    virtual void onInitialize() = 0;

    std::string name_;
    rclcpp::Node *node_;
  };
} // namespace sfframework

#endif // SFFRAMEWORK_PARTITIONING_STRATEGY_HPP_
