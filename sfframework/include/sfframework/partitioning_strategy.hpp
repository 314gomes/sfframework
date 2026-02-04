#ifndef SFFRAMEWORK_PARTITIONING_STRATEGY_HPP_
#define SFFRAMEWORK_PARTITIONING_STRATEGY_HPP_

#include <memory>
#include <string>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>
#include "sfframework/partitioning_context.hpp"
#include "sfframework/sfgenerator_utils.hpp"

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

      node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
      node_->declare_parameter(name_ + ".invert_selection", false);

      this->onInitialize();
    };

    void process(PartitioningContext &context)
    {
      this->indices_map_.clear();

      try
      {
        this->input_cloud_ = sfframework::SFGeneratorUtils::cloud_from_tags(
            context,
            node_->get_parameter(name_ + ".input_tags").as_string_array(),
            node_->get_parameter(name_ + ".invert_selection").as_bool(),
            this->indices_map_);
      }
      catch (const std::out_of_range &e)
      {
        RCLCPP_WARN(node_->get_logger(), "Partitioning plugin [%s] tried to find a list of clusters at a tag but it is missing. Skipping partitioner.", name_.c_str());
        return;
      }

      this->onProcess(context);
    }

  protected:
    // The logic step: Read input_cloud_ -> Process -> Write to Context
    virtual void onProcess(PartitioningContext &context) = 0;
    virtual void onInitialize() = 0;

    bool needsMapping() const
    {
      return !indices_map_.empty();
    }

    size_t mapToOriginalIndex(size_t index) const
    {
      if (indices_map_.empty())
      {
        return index;
      }
      return indices_map_[index];
    }

    std::string name_;
    rclcpp::Node *node_;
    std::shared_ptr<const open3d::geometry::PointCloud> input_cloud_;
    std::vector<size_t> indices_map_;
  };
} // namespace sfframework

#endif // SFFRAMEWORK_PARTITIONING_STRATEGY_HPP_
