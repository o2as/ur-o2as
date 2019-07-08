#include "o2as_vision_server.h"

VisionServer::VisionServer() : 
                  findObjectActionServer_(n_, "o2as_skills/findObject", boost::bind(&VisionServer::executeFindObject, this, _1),false)
{ 
  // Topics to publish

  // Services to advertise

  // Actions we serve
  findObjectActionServer_.start();
}

// ----------- Action servers

// findObjectAction
void VisionServer::executeFindObject(const o2as_vision::findObjectGoalConstPtr& goal)
{
  ROS_INFO("findObjectAction was called");
  // TODO: Insert commands to do the action
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ROS_INFO("findObjectAction is set as succeeded");
  findObjectActionServer_.setSucceeded();
}

// ----------- End of the class definitions

int main(int argc, char **argv)
{
  ros::init(argc, argv, "o2as_vision");

  // Create an object of class VisionServer that will take care of everything
  VisionServer o2as_vision_server;

  ROS_INFO("O2AS vision server started");

  ros::spin();
  return 0;
}
