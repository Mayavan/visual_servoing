/**
 * @file  UR5Control.cpp
 * @brief File With the definitions of the UR5Control class
 * methods.
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

#include "ur5_moveit_perception/ur5_control.hpp"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

UR5Control::UR5Control() : ur5_("manipulator") 
{
  ur5_.setPlannerId("ESTkConfigDefault");
  std::map<std::string, double> jointState;
  jointState.insert(std::pair< std::string, double>("shoulder_pan_joint", 0.00020395482828305944) ); 
  jointState.insert(std::pair< std::string, double>("shoulder_lift_joint", -1.4455796855988057)); 
  jointState.insert(std::pair< std::string, double>("elbow_joint", 1.6625897963983025)); 
  jointState.insert(std::pair<std::string, double>("wrist_1_joint", -0.22218163283984627)); 
  jointState.insert(std::pair<std::string, double>("wrist_2_joint", -5.904616108143301e-05)); 
  jointState.insert(std::pair<std::string, double>("wrist_3_joint", 2.304766549334488)); 

  // Wait 2 seconds before starting the demo
  ros::Duration(2).sleep();
  // Move to intermediate position
  ur5_.setJointValueTarget(jointState);
  ROS_INFO_STREAM("Moving to intermediate Position. Success: " << (bool) move());
  // Move to home position
  jointStateHome.insert(std::pair< std::string, double>("shoulder_pan_joint", 0.23646233514971593) ); 
  jointStateHome.insert(std::pair< std::string, double>("shoulder_lift_joint", -1.5498053448676172)); 
  jointStateHome.insert(std::pair< std::string, double>("elbow_joint", 1.7747350756382791)); 
  jointStateHome.insert(std::pair<std::string, double>("wrist_1_joint", -0.22126260656049115)); 
  jointStateHome.insert(std::pair<std::string, double>("wrist_2_joint", 1.8816210799697457)); 
  jointStateHome.insert(std::pair<std::string, double>("wrist_3_joint", 2.415634859502404)); 

  ros::Duration(2).sleep();
  ur5_.setJointValueTarget(jointStateHome);
  ROS_INFO_STREAM("Moving to Home Position. Success: " << (bool) move());
}

UR5Control::UR5Control(geometry_msgs::Pose pose) : ur5_("manipulator"), home(pose)
{
  ur5_.setPlannerId("ESTkConfigDefault");
  std::map<std::string, double> jointState;
  jointState.insert(std::pair< std::string, double>("shoulder_pan_joint", 0.00020395482828305944) ); 
  jointState.insert(std::pair< std::string, double>("shoulder_lift_joint", -1.4455796855988057)); 
  jointState.insert(std::pair< std::string, double>("elbow_joint", 1.6625897963983025)); 
  jointState.insert(std::pair<std::string, double>("wrist_1_joint", -0.22218163283984627)); 
  jointState.insert(std::pair<std::string, double>("wrist_2_joint", -5.904616108143301e-05)); 
  jointState.insert(std::pair<std::string, double>("wrist_3_joint", 2.304766549334488)); 

  // Wait 2 seconds before starting the demo
  ros::Duration(2).sleep();
  // Move to intermediate position
  ur5_.setJointValueTarget(jointState);
  ROS_INFO_STREAM("Moving to intermediate Position. Success: " << move());

  // Move to home position
  ur5_.setPoseTarget(home);
  move();
}

bool UR5Control::move() {
  auto current_time = ros::Time::now();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool success = (ur5_.plan(planner_) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    ur5_.execute(planner_);
  } else {
    return false;
  }
  return success;
}

bool UR5Control::grabFruit(geometry_msgs::Pose goal)
{
  // Calculate the offsetted position
  offset_goal = goal;
  offset_goal.position.x -= 0.1;

  // Move to the offsetted position
  ur5_.setPoseTarget(offset_goal);
  if(!move()) return 0;
  ROS_INFO_STREAM("Move to Offset");
  ros::Duration(4).sleep();

  // Move to the grabbing position
  ur5_.setPoseTarget(goal); 
  if(!move()) return 0;
  ROS_INFO_STREAM("Move to Goal");
  ros::Duration(4).sleep();

  // Move to the offsetted position again
  ur5_.setPoseTarget(offset_goal); 
  if(!move()) return 0;
  ROS_INFO_STREAM("Move to Offset");
  ros::Duration(4).sleep();

  // move back to home position
  ur5_.setJointValueTarget(jointStateHome);
  if(!move()) return 0;

  return true;
}