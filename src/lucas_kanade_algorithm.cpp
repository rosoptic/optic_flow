#include <lucas_kanade_algorithm.h>

namespace optic_flow
{
    LucasKanadeAlgorithm::LucasKanadeAlgorithm(bool debug) :
        tracked_elements_{0},
        found_elements_{0},
        do_initialize_{true},
        window_size_{31,31},
        points_{},
        sub_window_size_{10,10},
        end_criteria_{cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03},
        debug_{debug}
    {
    }

    void LucasKanadeAlgorithm::loadParams(ros::NodeHandle &priHandle)
    {
        ROS_INFO("Lucas Canade Params:");
        priHandle.param("lucas_kanade/corners_threshold", params_.corners_threshold, 0.4);
        ROS_INFO_STREAM("    corners_threshold: " << params_.corners_threshold);
        priHandle.param("lucas_kanade/max_flow_corners", params_.max_flow_corners, 500);
        ROS_INFO_STREAM("    max_flow_corners:  " << params_.max_flow_corners);
        priHandle.param("lucas_kanade/min_flow_corners", params_.max_flow_corners, 500);
        ROS_INFO_STREAM("    min_flow_corners:  " << params_.max_flow_corners);
    }

    cv_bridge::CvImage LucasKanadeAlgorithm::getDebugImage()
    {
        return cv_bridge::CvImage(std_msgs::Header(),"bgr8",debug_img_);
    }


    void LucasKanadeAlgorithm::process(const cv_bridge::CvImageConstPtr& img)
    {
        processFrame(img);
        do_initialize_ = isReadyToRecalculate();
    }

    void LucasKanadeAlgorithm::processFrame(const cv_bridge::CvImageConstPtr& img)
    {
        ROS_DEBUG("Procesing image" );
        cv::cvtColor(img->image, grey_, cv::COLOR_BGR2GRAY);
        if (debug_)
            img->image.copyTo(debug_img_);
        if (do_initialize_)
        {
            ROS_DEBUG("Initializing flow data");
            cv::goodFeaturesToTrack(grey_, points_[1], params_.max_flow_corners, 0.01,
                    10, cv::Mat{}, 3, false, 0.04);
            cv::cornerSubPix(grey_, points_[1], sub_window_size_, cv::Size{-1,
                    -1}, end_criteria_);
            tracked_elements_ = points_[1].size();
            found_elements_ = tracked_elements_;
            do_initialize_ = false;
            grey_.copyTo(prev_grey_);
            ROS_DEBUG_STREAM("Initialized corner vector: " << points_[1].size() << " found");
        }
        else if (!points_[0].empty())
        {
            std::vector<uchar> status;
            std::vector<float> err;
            if (prev_grey_.empty())
                grey_.copyTo(prev_grey_);
            ROS_DEBUG("Performing Optic Flow");
            cv::calcOpticalFlowPyrLK(prev_grey_, grey_, points_[0], points_[1],
                    status, err, window_size_, 3, end_criteria_, 0, 0.00);
            found_elements_ = 0;
            for (int i = 0; i < points_[1].size(); i++)
            {
                if (!status[i] )
                    continue;
                else
                    found_elements_++;

                if (debug_)
                    cv::circle(debug_img_, points_[1][i], 3, cv::Scalar(0,255,0), -1, 8);


            }
            ROS_DEBUG_STREAM("corners found = " << found_elements_ << " corners tracked = "  << tracked_elements_);
        }
        std::swap(points_[1], points_[0]);
        cv::swap(grey_, prev_grey_);
    }

    bool LucasKanadeAlgorithm::isReadyToRecalculate()
    {
        auto ratio = found_elements_/(double)tracked_elements_;
        ROS_DEBUG_STREAM("Checking if corners need recalculating: found = " <<
                found_elements_ << " targeted = " << tracked_elements_ << " ratio = " << ratio);
        auto recalc = false;
        if ( ratio < params_.corners_threshold)
        {
            ROS_DEBUG_STREAM("Corner Ratio Threshold Hit: target = "<<
                    params_.corners_threshold << " acutual = " << ratio);
            recalc = true;
        }
        else if (found_elements_ < params_.min_flow_corners)
        {
            ROS_DEBUG_STREAM("Min Detected Threshold Hit: target = " <<
                    params_.min_flow_corners << " actual = " << found_elements_);
            recalc= true;
        }
        return recalc;
    }

}
