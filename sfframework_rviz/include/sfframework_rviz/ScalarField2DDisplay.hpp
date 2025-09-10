#ifndef RVIZ_PLUGIN_SCALAR_FIELD_2D_DISPLAY_HPP_
#define RVIZ_PLUGIN_SCALAR_FIELD_2D_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <sfframework_msgs/msg/scalar_field2_d.hpp>
#include <rviz_rendering/objects/mesh_shape.hpp>


namespace ScalarField2DDisplay
{
class ScalarField2DDisplay
  : public rviz_common::MessageFilterDisplay<sfframework_msgs::msg::ScalarField2D>
{
  Q_OBJECT

protected:
  void onInitialize() override;
  void processMessage(const sfframework_msgs::msg::ScalarField2D::ConstSharedPtr msg) override;
  std::unique_ptr<rviz_rendering::MeshShape> resulting_mesh_;
};
}  // namespace ScalarField2DDisplay

#endif  // RVIZ_PLUGIN_SCALAR_FIELD_2D_DISPLAY_HPP_