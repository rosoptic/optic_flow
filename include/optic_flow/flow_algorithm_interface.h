#ifndef FLOW_ALGORITHM_INTERFACE_H
#define FLOW_ALGORITHM_INTERFACE_H
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
namespace optic_flow
{
    class FlowAlgorithmInterface
    {
        public:
            virtual void process(const cv_bridge::CvImageConstPtr& input) = 0;
            virtual void loadParams(ros::NodeHandle& priHandle) = 0;

            virtual cv_bridge::CvImage getDebugImage() = 0;
    };
}



#endif
