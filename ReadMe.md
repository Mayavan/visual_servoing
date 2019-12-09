# Visual Servoing

## Installing Dependencies

Install moveit if moveit is not already installed

```
sudo apt-get install ros-kinetic-moveit
```

## Instructions to setup the workspace to use with UR5 Robot

Go to the directory where you want the workspace and run the commands to create a catkin workspace.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

UR5 packages installation instruction from the ROS Industrial github repository:
```
cd $HOME/catkin_ws/src

# retrieve the sources (replace '$ROS_DISTRO' with the ROS version you are using)
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

# checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```

## Instructions to setup the Visual Servoing Package

```
cd ~/catkin_ws/src
git clone https://github.com/Mayavan/visual_servoing.git
cd ~/catkin_ws
catkin_make
```

## Instruction to run the demo

```
roslaunch visual_servoing visual_servo.launch 
```