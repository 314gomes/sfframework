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

      node_->declare_parameter(name_ + ".input_tags", std::vector<std::string>());
      node_->declare_parameter(name_ + ".invert_selection", false);

      this->onInitialize();
    };

    void process(PartitioningContext &context){
      this->indices_map_.clear();
      if (node_->get_parameter(name_ + ".input_tags").as_string_array().empty())
      {
        this->input_cloud_ = context.cloud;
      }
      else{
        std::vector<size_t> input_indices_;
        auto input_tags = node_->get_parameter(name_ + ".input_tags").as_string_array();
        for (const auto &tag : input_tags){
          std::vector<PartitioningCluster> clusters_at_tag;
          
          try
          {
            clusters_at_tag = context.clusters_registry.at(tag);
          }
          catch(const std::out_of_range& e)
          {
            RCLCPP_WARN(node_->get_logger(), "Partitioning plugin [%s] tried to find a list of clusters at tag \"%s\" but tag is missing. Skipping partitioner.", name_.c_str(), tag.c_str());
            return;
          }
          
          for (const auto &cluster : clusters_at_tag){
            input_indices_.insert(input_indices_.end(), cluster.indices.begin(), cluster.indices.end());
          }
        
        }

        if (node_->get_parameter(name_ + ".invert_selection").as_bool()) {
          size_t num_points = context.cloud->points_.size();
          std::vector<bool> mask(num_points, true);
          for (size_t idx : input_indices_) {
            if (idx < num_points) mask[idx] = false;
          }
          for (size_t i = 0; i < num_points; ++i) {
            if (mask[i]) this->indices_map_.push_back(i);
          }
        } else {
          this->indices_map_ = input_indices_;
        }
        this->input_cloud_ = context.cloud->SelectByIndex(this->indices_map_, false);
      }
      this->onProcess(context);
    }
    
    protected:
    // The logic step: Read input_cloud_ -> Process -> Write to Context
    virtual void onProcess(PartitioningContext &context) = 0;
    virtual void onInitialize() = 0;

    bool needsMapping() const {
      return !indices_map_.empty();
    }

    size_t mapToOriginalIndex(size_t index) const {
      if (indices_map_.empty()) {
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
