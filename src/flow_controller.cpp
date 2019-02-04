#include <flow_controller.h>

namespace optic_flow 
{
    const std::string FlowController::RAW_WINDOW = "raw image";

    FlowController::FlowController(ros::NodeHandle& publicHandler,
            ros::NodeHandle& privateHandle):
        pubHandle_{publicHandler}, priHandle_{privateHandle},
        params_{loadParams()},
        transport_{pubHandle_}, img_queue_{}  
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
            if (params_.display_cv_debug)
            {
                auto encoding = "bgr8";
                try
                {
                    cv::imshow(RAW_WINDOW, cv_bridge::toCvShare(frame, encoding)->image);
                    cv::waitKey(30);
                }
                catch(cv_bridge::Exception& e)
                {
                    ROS_ERROR_STREAM("Could not convert from " << frame->encoding.c_str() << "to "<< encoding);

                }
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
        OpticFlowParams params{};
        priHandle_.param("rate", params.rate, 30);
        priHandle_.param("display_cv_debug", params.display_cv_debug, false);
        ROS_INFO("Params:");
        ROS_INFO_STREAM("    rate:             " << params.rate);
        ROS_INFO_STREAM("    display_cv_debug: " << params.display_cv_debug);
        return params;
 
    }

    void FlowController::setupDebugWindows()
    {
        cv::namedWindow(RAW_WINDOW);
        cv::startWindowThread();
    }

    void FlowController::closeDebugWindows()
    {
        cv::destroyWindow(RAW_WINDOW);
        
    }


}
