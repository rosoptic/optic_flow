#ifndef FLOW_CONTROLLER_H
#define FLOW_CONTROLLER_H
#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
namespace optic_flow
{
    class FlowController
    {
        public:
            FlowController(ros::NodeHandle &publicHandle, ros::NodeHandle& privateHandle);
            ~FlowController();
            void enqueueFrame(const sensor_msgs::ImageConstPtr& img);
            void process();

            struct OpticFlowParams
            {
                int rate;
                bool display_cv_debug;
                OpticFlowParams() : rate{}, display_cv_debug{} {}
                OpticFlowParams(const OpticFlowParams& params) :
                    rate{params.rate}, display_cv_debug{params.display_cv_debug} {}
            };
            OpticFlowParams getParams();

        private:
            ros::NodeHandle pubHandle_;
            ros::NodeHandle priHandle_;
            image_transport::ImageTransport transport_;
            std::queue<sensor_msgs::ImageConstPtr> img_queue_;

            const OpticFlowParams params_;

            image_transport::Subscriber raw_sub_;

            const static int MAX_SIZE = 5;
            const static std::string RAW_WINDOW;

            FlowController::OpticFlowParams loadParams();

            void setupDebugWindows();
            void displayDebugWindows();
            void closeDebugWindows();

    };
}
#endif
