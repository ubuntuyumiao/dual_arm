#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <geometry_msgs/PoseStamped.h>


void callback(const apriltags_ros::AprilTagDetectionArrayConstPtr& msg)
{
  ros::NodeHandle nh;
  int tagnum=msg->detections.size();
  std::vector<apriltags_ros::AprilTagDetection> message=msg->detections;
  int ID;
  nh.getParam("/apriltag_detector/TragetID",ID);
  for(int i=0;i<tagnum;i++)
  {
    apriltags_ros::AprilTagDetection Detection=message.at(i);
    if(Detection.id==ID)
    {
       static tf::TransformBroadcaster br;
       tf::Transform transform;
       transform.setOrigin(tf::Vector3(Detection.pose.pose.position.x,Detection.pose.pose.position.y,Detection.pose.pose.position.z));
       ROS_INFO("look at result  %.2f, %.2f, %.2f",Detection.pose.pose.position.x,Detection.pose.pose.position.y,Detection.pose.pose.position.z);
       tf::Quaternion Q=tf::Quaternion( Detection.pose.pose.orientation.x,Detection.pose.pose.orientation.y,Detection.pose.pose.orientation.z, Detection.pose.pose.orientation.w);
       tf::Quaternion q=Q.normalized();

       transform.setRotation(q);
       //ROS_INFO("look at rotation  %.2f, %.2f, %.2f ,%.2f", q.x, q.y, q.z, q.w);
       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera","tag"));
       nh.setParam("Findtag_flag",true);
    }

  }
}
int main(int argc,char** argv)
{
  ros::init(argc,argv,"tag_broadcaster");
  ros::NodeHandle n;
  ros::Subscriber sub=n.subscribe<apriltags_ros::AprilTagDetectionArray>("/tag_detections",1,callback);
  ros::spin();
}
