#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string camera_link_name;

/*
void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", camera_link_name));
}
*/

void publishStaticCameraTF()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.9, 0.0, 1.035) );
  tf::Quaternion q;
  q.setRPY(0, 0, 3.14);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", camera_link_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_tf_broadcaster");

  ros::NodeHandle node;
  ros::Rate loop_rate(30);
  //ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  camera_link_name = "camera_link";
  while(ros::ok())
  {
    publishStaticCameraTF();
    loop_rate.sleep();
  }
  return 0;
};