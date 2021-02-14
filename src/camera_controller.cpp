#include <math.h>
#include "ros/ros.h"
#include "car_control_msgs/CameraControl.h"
#include "std_msgs/Header.h"
#include <tf/transform_broadcaster.h>
#include "headset_type.h"

ros::Publisher publisher;
HeadSetType type = HeadSetType::OculusQuest2;

void publishRPY(const std_msgs::Header &header, const double r, const double p, const double y) {
    car_control_msgs::CameraControl msg;
    msg.header = header;
    msg.roll = r;
    msg.pitch = p;
    msg.yaw = y;
    if (type == HeadSetType::OculusQuest2) {
      msg.roll = -msg.roll;
    }
    ROS_DEBUG("publish :  seq = [%d], r = %f, p = %f, y = %f", msg.header.seq, msg.roll, msg.pitch, msg.yaw);
    publisher.publish(msg);
}

void chatterCallback(const geometry_msgs::PoseStamped &msg) {
  geometry_msgs::Quaternion quaternion = msg.pose.orientation;
  double r,p,y;
  tf::Quaternion quat(quaternion.z, -quaternion.x, quaternion.y, -quaternion.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  ROS_DEBUG("convert : seq = [%d], r = %f, p = %f, y = %f", msg.header.seq, r * 180 / M_PI, p * 180 / M_PI, y * 180 / M_PI);
  publishRPY(msg.header, r * 180 / M_PI, p * 180 / M_PI, y * 180 / M_PI);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_controller");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  std::string type_str;
  std::string topic_name;
  pn.getParam("headset_type", type_str);
  if (type_str == "OculusQuest2") {
    type = HeadSetType::OculusQuest2;
    topic_name = "/oculus/head_set/pose";
  } else {
    type = HeadSetType::OculusGo;
    topic_name = "oculus_pose";
  }

  publisher = n.advertise<car_control_msgs::CameraControl>("camera_rpy", 100);

  ros::Subscriber subscriber = n.subscribe(topic_name, 50, chatterCallback);

  ros::spin();

  return 0;
}