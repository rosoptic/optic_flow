#include <ros/ros.h>
#include <flow_controller.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "optic_flow_node");
  ros::NodeHandle nh{};
  ros::NodeHandle pnh{"~"};
  optic_flow::FlowController controller{nh, pnh};
  auto params = controller.getParams();
  auto rate = ros::Rate(params.rate);
  while (ros::ok())
  {
    ros::spinOnce();
    controller.process();
    rate.sleep();
  }
}
