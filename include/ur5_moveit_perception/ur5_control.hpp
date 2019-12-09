/**
 * @file  UR5Control.hpp
 * @brief File With the declarations of the UR5Control class
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

#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

/**
 * @brief Class UR5Control has method to send goal pose of the
 * ur5 using moveit. 
 */
class UR5Control
{
private:
    /**
     * @brief MoveGroupInterface object to manage the ur5 robot.
     */
    moveit::planning_interface::MoveGroupInterface ur5_;

    /**
     * @brief Plan planned path to be executed by ur5 robot.
     */
    moveit::planning_interface::MoveGroupInterface::Plan planner_;

    /**
     * @brief Pose to calculate the of offset pose of gripper before plucking or grabbing.
     */
    geometry_msgs::Pose offset_goal;

    /**
     * @brief Pose to stay when the robot is moved around in the greenhouse.
     */
    geometry_msgs::Pose home;

    /**
     * @brief Joint State to stay when the robot is moved around in the greenhouse.
     */
    std::map<std::string, double> jointStateHome;

    /**
     * @brief Function to execute the set target.
     * @return true if successful else false
     */
    bool move();

public:
    /**
     * @brief Default constructer for the UR5Control class
     */
    UR5Control();

    /**
     * @brief Constructer for the UR5Control class with custom home position
     */
    UR5Control(geometry_msgs::Pose);

    /**
     * @brief Function to move the gripper to the offset 
     * position, move to grabbing position,
     * back to offset and home position.
     * @param goal or the position to reach to grab the crop
     * @return true if the execution was successful else false
     */
    bool grabFruit(geometry_msgs::Pose);
};