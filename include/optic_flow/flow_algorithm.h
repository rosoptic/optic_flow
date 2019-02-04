#ifndef FLOW_ALGORITHM_H
#define FLOW_ALGORITHM_H
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/ros.h"

namespace optic_flow
{
    class FlowAlgorithm
    {
        public:
            FlowAlgorithm(int flow_corners);
            cv::Mat process(const cv::Mat& input);
            void triggerInitialize();
            std::tuple<int, int> getFoundVTracked();

        private:
            int max_flow_corners_;
            int tracked_elements_;
            int found_elements_;
            bool do_initialize_;
            std::vector<cv::Point2f> points_[2];
            cv::Size sub_window_size_;
            cv::Size window_size_;
            cv::TermCriteria end_criteria_;
            cv::Mat img_;
            cv::Mat grey_;
            cv::Mat prev_grey_;

    };
}



#endif
