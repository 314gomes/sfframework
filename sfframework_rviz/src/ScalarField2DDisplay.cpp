#include <sfframework_rviz/ScalarField2DDisplay.hpp>
#include <rviz_common/logging.hpp>

namespace ScalarField2DDisplay
{
void ScalarField2DDisplay::processMessage(const sfframework_msgs::msg::ScalarField2D::ConstSharedPtr msg)
{
  // RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);
  // TODO: check if the message is valid
    
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  // Get the transform from the message frame to the fixed frame
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    RVIZ_COMMON_LOG_WARNING_STREAM("Failed to transform from frame " << msg->header.frame_id << " to fixed frame");
    return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);


  int n_vertices = msg.get()->info.width * msg.get()->info.height;
  // int n_triangles = 2;

  
  // clear previous mesh
  this->resulting_mesh_->clear();
  
  this->resulting_mesh_->estimateVertexCount(n_vertices);
  
  for (std::size_t i = 0 ; i < msg.get()->data.size() ; ++i)
  {
    float x = (i % msg.get()->info.width) * msg.get()->info.resolution + msg.get()->info.origin.position.x;
    float y = (i / msg.get()->info.width) * msg.get()->info.resolution + msg.get()->info.origin.position.y;
    float z = msg.get()->data[i];
    this->resulting_mesh_->addVertex(Ogre::Vector3(x, y, z));
  }
  
  this->resulting_mesh_->beginTriangles();
  
  for (std::size_t i = 0 ; i < msg.get()->info.height - 1 ; ++i)
  {
    for (std::size_t j = 0 ; j < msg.get()->info.width - 1 ; ++j)
    {
      std::size_t v1 = i * msg.get()->info.width + j;
      std::size_t v2 = v1 + 1;
      std::size_t v3 = v1 + msg.get()->info.width;
      std::size_t v4 = v3 + 1;

      this->resulting_mesh_->addTriangle(v1, v2, v3);
      this->resulting_mesh_->addTriangle(v2, v4, v3);
    }
  }



  // for (std::size_t i = 0 ; i < triangles.size() ; ++i)
  //   mesh->addTriangle(triangles[i].v1, triangles[i].v2, triangles[i].v3);
  
  
  this->resulting_mesh_->endTriangles();

}

void ScalarField2DDisplay::onInitialize()
{
  MFDClass::onInitialize();
  
  resulting_mesh_ = std::make_unique<rviz_rendering::MeshShape>(scene_manager_, scene_node_);
}

}  // namespace ScalarField2DDisplay

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ScalarField2DDisplay::ScalarField2DDisplay, rviz_common::Display)