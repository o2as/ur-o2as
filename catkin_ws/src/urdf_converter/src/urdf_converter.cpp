#include "urdf_converter.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_urdf_converter");

  std::string path = ros::package::getPath("o2as_parts_description") + "/urdf/generated/collision_object_urdfs";
  std::vector<moveit_msgs::CollisionObject> messages = convertURDFFilesToCollisionObjectMsg(path);

  return 1;
}
