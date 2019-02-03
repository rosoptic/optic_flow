#ifndef FLOW_CONTROLLER_H
#define FLOW_CONTROLLER_H
#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Image.h>

namespace optic_flow
{
    class FlowController
    {
        public:
            FlowController();
            ~FlowController();
            void enqueueFrame(const sensor_msgs::ImageConstPtr& img);
            void process();
        private:
            std::queue<sensor_msgs::ImageConstPtr> img_queue_;
            const static int MAX_SIZE = 5;

    };
}
#endif
