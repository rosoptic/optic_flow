#ifndef FARNEBACK_ALGORITHM_H
#define FARNEBACK_ALGORITHM_H
#include "flow_algorithm_interface.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/video/tracking.hpp"

namespace optic_flow
{
    class FarnebackAlgorithm : public FlowAlgorithmInterface
    {
        public:
            FarnebackAlgorithm(bool debug);
            void process(const cv_bridge::CvImageConstPtr& input) override;
            void loadParams(ros::NodeHandle& priHandle) override; 
            cv_bridge::CvImage getDebugImage() override;

        private:
            struct FarnebackParams
            {
                double pyr_scale;
                int pyr_levels;
                int win_size;
                int iterations;
                int poly_n;
                double poly_sigma;
                int debug_grid;
                FarnebackParams(): 
                    pyr_scale{},
                    pyr_levels{},
                    win_size{},
                    iterations{},
                    poly_n{},
                    poly_sigma{},
                    debug_grid{}
                {}
            };
 
            cv::Mat grey_;
            cv::Mat prev_grey_;
            cv::Mat flow_;
            cv::UMat uFlow_;
            cv::Mat debug_img_;
            bool debug_;
            FarnebackParams params_;
    };
}

#endif
