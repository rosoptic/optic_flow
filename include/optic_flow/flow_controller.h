#ifndef FLOW_CONTROLLER_H
#define FLOW_CONTROLLER_H
#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <flow_algorithm_interface.h>
#include <lucas_kanade_algorithm.h>
namespace optic_flow
{
    class FlowController
    {
        public:
            FlowController(ros::NodeHandle &publicHandle, ros::NodeHandle& privateHandle);
            ~FlowController();
            void enqueueFrame(const sensor_msgs::ImageConstPtr& img);
            void process();
            double getRate();
        private:
            struct OpticFlowParams
            {
                double rate;
                bool publish_cv_debug;
                std::string flow_algorithm;
                OpticFlowParams() : 
                    rate{}, 
                    publish_cv_debug{},
                    flow_algorithm{}
                {}
            };

            ros::NodeHandle pubHandle_;
            ros::NodeHandle priHandle_;
            image_transport::ImageTransport transport_;
            std::queue<sensor_msgs::ImageConstPtr> img_queue_;

            OpticFlowParams params_;

            image_transport::Subscriber raw_sub_;
            image_transport::Publisher debug_pub_;


            std::unique_ptr<FlowAlgorithmInterface> algorithm_ptr_;
            const static int MAX_SIZE = 5;
            const static std::string RAW_WINDOW;
            const static std::string FLOW_WINDOW;

            void loadParams();
            cv_bridge::CvImageConstPtr convertToCvPtr(sensor_msgs::ImageConstPtr& image);

            void setupDebugWindows();
            void displayDebugWindows();
            void closeDebugWindows();

    };
}
#endif
