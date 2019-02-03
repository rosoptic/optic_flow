#include <ros/ros.h>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_DEBUG_STREAM("Recieved Message time => " << msg->header.stamp);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optic_flow_node");
  ros::NodeHandle nh{};
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
  ros::spin();
}
