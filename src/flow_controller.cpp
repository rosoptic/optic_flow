#include <flow_controller.h>

namespace optic_flow 
{
    const std::string FlowController::RAW_WINDOW = "raw image";
    const std::string FlowController::FLOW_WINDOW = "flow image";

    FlowController::FlowController(ros::NodeHandle& publicHandler,
            ros::NodeHandle& privateHandle):
        pubHandle_{publicHandler}, priHandle_{privateHandle},
        params_{loadParams()},
        transport_{pubHandle_}, img_queue_{},
        flow_algorithm_{params_.max_flow_corners}
    {
        ROS_INFO("Constructing FlowController");
        raw_sub_ = transport_.subscribe("image", 5,&FlowController::enqueueFrame, this);
        if (params_.display_cv_debug)
        {
            setupDebugWindows();
        }
    }

    FlowController::~FlowController()
    {
        ROS_INFO("Destructing FlowController");
        if (params_.display_cv_debug)
        {
            closeDebugWindows();
        }
    }


    void FlowController::enqueueFrame(const sensor_msgs::ImageConstPtr& img)
    {
	    ROS_DEBUG_STREAM("Recieved Message time => " << img->header.stamp);
	    img_queue_.push(img);
    }

    cv_bridge::CvImageConstPtr FlowController::convert(sensor_msgs::ImageConstPtr& img)
    {
        auto encoding = "bgr8";
        cv_bridge::CvImageConstPtr  cvImg;
        try
        {
            cvImg = cv_bridge::toCvShare(img, encoding);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR_STREAM("Could not convert from " << img->encoding.c_str() << "to "<< encoding);

        }
        return cvImg;
    }


    void FlowController::process()
    {
	    ROS_DEBUG_STREAM("Processing img_queue_ size: " << img_queue_.size());
        

        if (!img_queue_.empty())
        {
            if (img_queue_.size() > MAX_SIZE)
            {
                ROS_WARN("The image queue is recieving images faster than it can process.");
            }
            auto frame = img_queue_.front();
            auto cvImg =  convert(frame);
            auto output = flow_algorithm_.process(cvImg->image);
            auto thresholds = flow_algorithm_.getFoundVTracked();
            auto found = std::get<0>(thresholds);
            auto targeted = std::get<1>(thresholds);
            auto ratio = found/(double)targeted;
            ROS_DEBUG_STREAM("Checking if corners need recalculating: found = " << found <<
                    " targeted = " << targeted << " ratio = " << ratio);
            bool reinitialize = false;
            if ( ratio < params_.corners_threshold)
            {
                ROS_DEBUG_STREAM("Corner Ratio Threshold Hit: target = "<<
                        params_.corners_threshold << " acutual = " << ratio);
                reinitialize = true;
            }
            else if (found < params_.min_flow_corners)
            {
                ROS_DEBUG_STREAM("Min Detected Threshold Hit: target = " <<
                        params_.min_flow_corners << " actual = " << found);
                reinitialize = true;
            }
            if (reinitialize)
            {
                flow_algorithm_.triggerInitialize();
            }
            if (params_.display_cv_debug)
            {
                cv::imshow(RAW_WINDOW, cvImg->image);
                cv::imshow(FLOW_WINDOW, output);
                cv::waitKey(30);
            }
	        ROS_DEBUG_STREAM("Popping Message time => " << frame->header.stamp);
            img_queue_.pop();

        }
        
    }


    FlowController::OpticFlowParams FlowController::getParams()
    {
        return params_;
 
    }

    FlowController::OpticFlowParams FlowController::loadParams()
    {
        ROS_INFO("Params:");
        OpticFlowParams params{};
        priHandle_.param("rate", params.rate, 30);
        ROS_INFO_STREAM("    rate:              " << params.rate);
        priHandle_.param("corners_threshold", params.corners_threshold, 0.4);
        ROS_INFO_STREAM("    corners_threshold: " << params.corners_threshold);
        priHandle_.param("max_flow_corners", params.max_flow_corners, 500);
        ROS_INFO_STREAM("    max_flow_corners:  " << params.max_flow_corners);
        priHandle_.param("display_cv_debug", params.display_cv_debug, false);
        ROS_INFO_STREAM("    display_cv_debug:  " << params.display_cv_debug);
        return params;
 
    }

    void FlowController::setupDebugWindows()
    {
        cv::namedWindow(RAW_WINDOW);
        cv::namedWindow(FLOW_WINDOW);
        cv::startWindowThread();
    }

    void FlowController::closeDebugWindows()
    {
        cv::destroyWindow(RAW_WINDOW);
        cv::destroyWindow(FLOW_WINDOW);
        
        
    }


}
