#include <urdf/model.h>
#include "urdf_parser/urdf_parser.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <moveit_msgs/CollisionObject.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

// Based on check_urdf in urdfdom
// https://github.com/ros/urdfdom_headers/blob/master/urdf_model/include/urdf_model/link.h
// https://github.com/ros-planning/geometric_shapes/blob/kinetic-devel/include/geometric_shapes/mesh_operations.h

moveit_msgs::CollisionObject makeCollObjMsg(urdf::LinkConstSharedPtr link)
{
  moveit_msgs::CollisionObject co;
  co.id = link->name;
  // Maybe this can be redone later for more than one shape
  // co.primitives.resize(link->child_links.size()); 
  // co.primitive_poses.resize(link->child_links.size());
  co.primitives.resize(1);
  co.primitive_poses.resize(1);
  auto shape = link->visual;
  co.primitives[0].type = link->visual->geometry->type;
  co.primitive_poses[0].position.x = link->visual->origin.position.x;
  co.primitive_poses[0].position.y = link->visual->origin.position.x;
  co.primitive_poses[0].position.z = link->visual->origin.position.x;
  co.primitive_poses[0].orientation.x = link->visual->origin.rotation.x;
  co.primitive_poses[0].orientation.y = link->visual->origin.rotation.y;
  co.primitive_poses[0].orientation.z = link->visual->origin.rotation.z;
  co.primitive_poses[0].orientation.w = link->visual->origin.rotation.w;
  co.mesh_poses.resize(1);
  co.mesh_poses[0] = co.primitive_poses[0];
  if (link->visual->geometry->type == urdf::Geometry::SPHERE)
  {
    co.primitives[0].dimensions.resize(1);
    co.primitives[0].dimensions[0] = dynamic_cast<urdf::Sphere&>(*(link->visual->geometry)).radius;
  }
  else if (link->visual->geometry->type == urdf::Geometry::BOX)
  {
    co.primitives[0].dimensions.resize(3);
    co.primitives[0].dimensions[0] = dynamic_cast<urdf::Box&>(*(link->visual->geometry)).dim.x;
    co.primitives[0].dimensions[1] = dynamic_cast<urdf::Box&>(*(link->visual->geometry)).dim.y;
    co.primitives[0].dimensions[2] = dynamic_cast<urdf::Box&>(*(link->visual->geometry)).dim.z;
  }
  else if (link->visual->geometry->type == urdf::Geometry::CYLINDER)
  {
    co.primitives[0].dimensions.resize(2);
    co.primitives[0].dimensions[0] = dynamic_cast<urdf::Cylinder&>(*(link->visual->geometry)).length;
    co.primitives[0].dimensions[1] = dynamic_cast<urdf::Cylinder&>(*(link->visual->geometry)).radius;
  }
  else 
  {
    co.primitives.resize(0);
    co.primitive_poses.resize(0);
  }
  
  if (link->visual->geometry->type == urdf::Geometry::MESH)
  {
    co.meshes.resize(1);
    std::string filename = dynamic_cast<urdf::Mesh&>(*(link->visual->geometry)).filename;
    shapes::Mesh* m = shapes::createMeshFromResource(filename);
    shape_msgs::Mesh m_msg;
    shapes::ShapeMsg s_msg = m_msg;
    shapes::constructMsgFromShape(m,s_msg);
    co.meshes[0] = m_msg;
  }
  else 
  {
    co.mesh_poses.resize(0);
  }

  int count = 0;
  co.frame_names.resize(link->child_joints.size());
  co.named_frame_poses.resize(link->child_joints.size());
  for (std::vector<urdf::JointSharedPtr>::const_iterator child = link->child_joints.begin(); child != link->child_joints.end(); child++)
  {
    co.frame_names[count] = (*child)->child_link_name;
    co.named_frame_poses[count].position.x = (*child)->parent_to_joint_origin_transform.position.x;
    co.named_frame_poses[count].position.y = (*child)->parent_to_joint_origin_transform.position.x;
    co.named_frame_poses[count].position.z = (*child)->parent_to_joint_origin_transform.position.x;
    co.named_frame_poses[count].orientation.x = (*child)->parent_to_joint_origin_transform.rotation.x;
    co.named_frame_poses[count].orientation.y = (*child)->parent_to_joint_origin_transform.rotation.y;
    co.named_frame_poses[count].orientation.z = (*child)->parent_to_joint_origin_transform.rotation.z;
    co.named_frame_poses[count].orientation.w = (*child)->parent_to_joint_origin_transform.rotation.w;
    count++;
  }
}

moveit_msgs::CollisionObject convertURDFFileToCollisionObjectMsg(std::string filepath)
{
  // The parseURDF function can be used if the source is a string from the parameter server (like robot_description).
  urdf::ModelInterfaceSharedPtr robot = urdf::parseURDFFile(filepath);  
  if (!robot){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return moveit_msgs::CollisionObject();
  }
  std::cout << "collision object name is: " << robot->getName() << std::endl;

  urdf::LinkConstSharedPtr root_link = robot->getRoot();
  if (!root_link)
  {
    ROS_ERROR("Object has no root link. It needs one root link to be converted to a collision object message.");
  }
  return makeCollObjMsg(root_link);
}

std::vector<moveit_msgs::CollisionObject> convertURDFFilesToCollisionObjectMsg(std::string directory)
{
  // Read input files (https://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c)
  // Input files need to be pure URDF files (not xacro)
  std::vector<moveit_msgs::CollisionObject> v;

  DIR *dir;
  struct dirent *ent;
  dir = opendir(directory.c_str());
  if (dir != NULL)
  {
    while ((ent = readdir (dir)) != NULL) {
      std::string urdf_file = ent->d_name;
      moveit_msgs::CollisionObject msg = convertURDFFileToCollisionObjectMsg(directory + "/" + urdf_file);
      
      v.push_back(msg);
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
  }
  return v;
}
