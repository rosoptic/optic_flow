#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <flow_controller.h>
#include <params.h>

using namespace optic_flow;

FlowController g_controller{};

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	g_controller.enqueueFrame(msg);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "optic_flow_node");
  ros::NodeHandle nh{};
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 5, imageCallback);
  OpticFlowParams params = getParams();
  auto rate = ros::Rate(params.rate);
  while (ros::ok())
  {
    ros::spinOnce();
    g_controller.process();
    rate.sleep();

  }
}
