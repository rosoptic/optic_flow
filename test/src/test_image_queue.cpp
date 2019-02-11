#include "image_queue.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// Bring in gtest
#include <gtest/gtest.h>


class ImageQueueFixture : public ::testing::Test
{
    protected:
        sensor_msgs::ImageConstPtr imgA;
        sensor_msgs::ImageConstPtr imgB;
        optic_flow::ImageQueue iq;



        ImageQueueFixture():
            iq{5,3}
        {
            imgA = sensor_msgs::ImageConstPtr{new sensor_msgs::Image{}};
            imgB = sensor_msgs::ImageConstPtr{new sensor_msgs::Image{}};

        }

};

TEST_F(ImageQueueFixture, ConstructorTest)
{
    try
    {
        optic_flow::ImageQueue inv{0, 0};
        FAIL();
    }
    catch (const std::invalid_argument& ia) {}
    try
    {
        optic_flow::ImageQueue inv{1, 0};
        FAIL();
    }
    catch (const std::invalid_argument& ia) {}
    try
    {
        optic_flow::ImageQueue inv{0, 1};
        FAIL();
    }
    catch (const std::invalid_argument& ia) {}
}

TEST_F(ImageQueueFixture, PopEmptyTest)
{
    try
    {
        ASSERT_EQ(iq.size(),0);
        iq.popFrame();
        FAIL();
    }
    catch (const std::runtime_error& ia) {}

    iq.pushFrame(imgA);
    ASSERT_EQ(iq.size(), 1);
    auto out = iq.popFrame();
    ASSERT_EQ(out, imgA);
    ASSERT_EQ(iq.size(), 0);
}

TEST_F(ImageQueueFixture, AutoPopTest)
{
    ASSERT_NE(imgA, imgB);
    for (int i = 0;  i < 5; i++)
    {
        if (i % 2)
            iq.pushFrame(imgA);
        else
            iq.pushFrame(imgB);
    }
    ASSERT_EQ(iq.size(), 5);
    auto front = iq.popFrame();
    ASSERT_EQ(front, imgB);
    ASSERT_EQ(iq.size(), 4);
    iq.pushFrame(imgA);
    iq.pushFrame(imgB);
    ASSERT_EQ(iq.size(), 6);
    front = iq.popFrame();
    ASSERT_EQ(front, imgB);
    ASSERT_EQ(iq.size(), 4);
    iq.pushFrame(imgA);
    iq.pushFrame(imgB);
    iq.pushFrame(imgA);
    iq.pushFrame(imgB);
    iq.pushFrame(imgA);
    ASSERT_EQ(iq.size(), 9);
    front = iq.popFrame();
    ASSERT_EQ(front, imgB);
    ASSERT_EQ(iq.size(), 5);
    



}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
