#include "image_queue.h"
#include <ros/ros.h>
// Bring in gtest
#include <gtest/gtest.h>

TEST(ImageQueue, ConstructorTest)
{
    optic_flow::ImageQueue im{};
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
