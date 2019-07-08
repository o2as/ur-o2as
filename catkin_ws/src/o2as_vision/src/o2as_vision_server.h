

#ifndef O2AS_VISION_SERVER_H
#define O2AS_VISION_SERVER_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"

#include <tf/transform_listener.h>    // Includes the TF conversions

#include <chrono>
#include <thread>

// Services

// Actions
#include <actionlib/server/simple_action_server.h>
#include "o2as_vision/findObjectAction.h"

class VisionServer
{
public:
  //Constructor
  VisionServer();

  //Helpers

  // Callback declarations

  // Actions
  void executeFindObject(const o2as_vision::findObjectGoalConstPtr& goal);
  

private:
  ros::NodeHandle n_;

  // Service declarations
  
  // Action declarations
  actionlib::SimpleActionServer<o2as_vision::findObjectAction> findObjectActionServer_;

  // Status variables

};//End of class VisionServer

#endif //O2AS_VISION_SERVER_H
