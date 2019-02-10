#include <ros/ros.h>
#include <flow_controller.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "optic_flow_node");
  ros::NodeHandle nh{};
  ros::NodeHandle pnh{"~"};
  optic_flow::FlowController controller{nh, pnh};
  auto rate = ros::Rate(controller.getRate());
  while (ros::ok())
  {
    ros::spinOnce();
    controller.process();
    rate.sleep();
  }
}
