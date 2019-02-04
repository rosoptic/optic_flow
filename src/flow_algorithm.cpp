#include <flow_algorithm.h>

namespace optic_flow
{
    FlowAlgorithm::FlowAlgorithm(int max_corners) :
        max_flow_corners_{max_corners},
        tracked_elements_{0},
        found_elements_{0},
        do_initialize_{true},
        window_size_{31,31},
        points_{},
        sub_window_size_{10,10},
        end_criteria_{cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03}
    {
    }

    void FlowAlgorithm::triggerInitialize()
    {
        ROS_DEBUG("Reinitialization Triggered");
        do_initialize_ = true;
    }

    std::tuple<int, int> FlowAlgorithm::getFoundVTracked()
    {
        return std::tuple<int, int>{std::make_tuple(found_elements_, tracked_elements_)};
    }


    cv::Mat FlowAlgorithm::process(const cv::Mat& input)
    {
        ROS_DEBUG("Procesing image" );
        cv::Mat img;
        input.copyTo(img);
        cv::cvtColor(img, grey_, cv::COLOR_BGR2GRAY);
        if (do_initialize_)
        {
            ROS_DEBUG("Initializing flow data");
            cv::goodFeaturesToTrack(grey_, points_[1], max_flow_corners_, 0.01,
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

                cv::circle(grey_, points_[1][i], 3, cv::Scalar(0,255,0), -1, 8);

            }
            ROS_DEBUG_STREAM("corners found = " << found_elements_ << " corners tracked = "  << tracked_elements_);
        }
        std::swap(points_[1], points_[0]);
        cv::swap(grey_, prev_grey_);
        return grey_;
    }
}
