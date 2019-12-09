/**
 * @file main.cpp
 * @brief Node to call the service to get a goal and execute required the trajectory
 *
 * @author RajendraMayavan
 * @copyright MIT License
 * 
 * Copyright (c) 2019 Mayavan
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ur5_moveit_perception/getTarget.h"
#include "ur5_moveit_perception/ur5_control.hpp"
#include "ros/ros.h"

#include <ur5_moveit_perception/ARMarker.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_planner");
 
  // Node Initialization
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ur5_moveit_perception::getTarget>("ar_pose_service");

  // Call service to get the goal position
  ur5_moveit_perception::getTarget srv;
  if (client.call(srv))
  {
    ROS_INFO_STREAM("Received Position: " << srv.response.msg.pose.pose.position.x << " " << srv.response.msg.pose.pose.position.y << " " << srv.response.msg.pose.pose.position.z);
  }
  else
  {
    ROS_ERROR("Failed to call service ar_pose_service");
    return 1;
  }

  // Execute the trajectory
  UR5Control controller;
  if(!controller.grabFruit(srv.response.msg.pose.pose)) return 1;

  ros::spin();
  return 0;
}
