#ifndef FLOW_CONTROLLER_H
#define FLOW_CONTROLLER_H
#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <flow_algorithm.h>
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
                double corners_threshold;
                int max_flow_corners;
                int min_flow_corners;
                OpticFlowParams() : rate{}, display_cv_debug{},
                    corners_threshold{}, max_flow_corners{},
                    min_flow_corners{}
                {}
                OpticFlowParams(const OpticFlowParams& params) :
                    rate{params.rate},
                    display_cv_debug{params.display_cv_debug},
                    corners_threshold{params.corners_threshold},
                    max_flow_corners{params.max_flow_corners},
                    min_flow_corners{params.min_flow_corners}
                {}
            };
            OpticFlowParams getParams();

        private:
            ros::NodeHandle pubHandle_;
            ros::NodeHandle priHandle_;
            image_transport::ImageTransport transport_;
            std::queue<sensor_msgs::ImageConstPtr> img_queue_;

            const OpticFlowParams params_;

            image_transport::Subscriber raw_sub_;
            FlowAlgorithm flow_algorithm_;
            const static int MAX_SIZE = 5;
            const static std::string RAW_WINDOW;
            const static std::string FLOW_WINDOW;

            FlowController::OpticFlowParams loadParams();
            cv_bridge::CvImageConstPtr convert(sensor_msgs::ImageConstPtr& image);

            void setupDebugWindows();
            void displayDebugWindows();
            void closeDebugWindows();

    };
}
#endif
