#include <farneback_algorithm.h>

namespace optic_flow
{
    FarnebackAlgorithm::FarnebackAlgorithm(bool debug):
        grey_{},
        prev_grey_{},
        flow_{},
        uFlow_{},
        debug_img_{},
        debug_{debug}

    {
    }

    void FarnebackAlgorithm::process(const cv_bridge::CvImageConstPtr& img)
    {
        ROS_DEBUG("Procesing image" );
        cv::cvtColor(img->image, grey_, cv::COLOR_BGR2GRAY);
        img->image.copyTo(debug_img_);
        if (prev_grey_.empty())
            grey_.copyTo(prev_grey_);
        ROS_DEBUG("Performing Optic Flow");
        auto flags = 0;
        
        cv::calcOpticalFlowFarneback(prev_grey_, grey_, uFlow_, 
                params_.pyr_scale, 
                params_.pyr_levels,
                params_.win_size, 
                params_.iterations, 
                params_.poly_n, 
                params_.poly_sigma,
                flags);
        uFlow_.copyTo(flow_);
        cv::swap(grey_, prev_grey_);
        if (debug_)
        {
            auto grid_size = 10;
            for (int y = 0; y < debug_img_.rows; y += params_.debug_grid)
            {
                for (int x = 0; x < debug_img_.cols; x += params_.debug_grid)
                {
                    auto flowAt = flow_.at<cv::Point2f>(y,x)*10;
                    cv::line(debug_img_, cv::Point(x, y), 
                            cv::Point(cvRound(x + flowAt.x), 
                                cvRound(y + flowAt.y)), cv::Scalar(255,0,0));
                    cv::circle(debug_img_, cv::Point(x,y), 1, cv::Scalar(0,0,0), -1);
                }
            }

        }
    }

    void FarnebackAlgorithm::loadParams(ros::NodeHandle& priHandle) 
    {
        ROS_INFO("Farneback Params:");
        priHandle.param("farneback/pyr_scale", params_.pyr_scale, 0.5);
        ROS_INFO_STREAM("    pyr_scale:  " << params_.pyr_scale);
        priHandle.param("farneback/pyr_levels", params_.pyr_levels, 1);
        ROS_INFO_STREAM("    pyr_levels: " << params_.pyr_levels);
        priHandle.param("farneback/win_size", params_.win_size, 5);
        ROS_INFO_STREAM("    win_size:   " << params_.win_size);
        priHandle.param("farneback/iterations", params_.iterations, 5);
        ROS_INFO_STREAM("    iterations: " << params_.iterations);
        priHandle.param("farneback/poly_n", params_.poly_n, 5);
        ROS_INFO_STREAM("    poly_n:     " << params_.poly_n);
        priHandle.param("farneback/poly_sigma", params_.poly_sigma, 1.1);
        ROS_INFO_STREAM("    poly_sigma: " << params_.poly_sigma);
        priHandle.param("farneback/debug_grid", params_.debug_grid, 10);
        ROS_INFO_STREAM("    debug_grid: " << params_.debug_grid);

     }
    cv_bridge::CvImage FarnebackAlgorithm::getDebugImage() 
    {
        return cv_bridge::CvImage(std_msgs::Header(),"bgr8",debug_img_);
    }


}
