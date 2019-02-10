#ifndef LUCAS_KANADE_ALGORITHM_H
#define LUCAS_KANADE_ALGORITHM_H
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/ros.h"
#include "flow_algorithm_interface.h"
namespace optic_flow
{

    class LucasKanadeAlgorithm : public FlowAlgorithmInterface
    {
        public:
            LucasKanadeAlgorithm(bool debug);
            void process(const cv_bridge::CvImageConstPtr& input) override;
            void loadParams(ros::NodeHandle& priHandle) override;
            cv_bridge::CvImage getDebugImage() override;
        private:
            struct LucasKanadeParams
            {
                double corners_threshold;
                int max_flow_corners;
                int min_flow_corners;
                LucasKanadeParams(): 
                    corners_threshold{}, 
                    max_flow_corners{},
                    min_flow_corners{}
                {}
            };
            LucasKanadeParams params_;
            int tracked_elements_;
            int found_elements_;
            bool do_initialize_;
            bool debug_;
            std::vector<cv::Point2f> points_[2];
            cv::Size sub_window_size_;
            cv::Size window_size_;
            cv::TermCriteria end_criteria_;
            cv::Mat img_;
            cv::Mat grey_;
            cv::Mat prev_grey_;
            cv::Mat debug_img_;


            void processFrame(const cv_bridge::CvImageConstPtr& input);
            bool isReadyToRecalculate();

    };
}
#endif

