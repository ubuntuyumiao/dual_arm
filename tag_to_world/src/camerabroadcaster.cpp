#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc,char** argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(1, 0, 0, 0), tf::Vector3(0, 0.16, 0.005)),
        ros::Time::now(),"base_link", "camera"));//care about the relationship about the base_link and camera
    broadcaster.sendTransform(
       tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 1)),
        ros::Time::now(),"world", "base_link"));
    r.sleep();
  }
  return 0;
}
