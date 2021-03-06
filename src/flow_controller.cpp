#include <flow_controller.h>

namespace optic_flow 
{
    const std::string FlowController::RAW_WINDOW = "raw image";
    const std::string FlowController::FLOW_WINDOW = "flow image";

    FlowController::FlowController(ros::NodeHandle& publicHandler,
            ros::NodeHandle& privateHandle):
        pubHandle_{publicHandler}, 
        priHandle_{privateHandle},
        params_{},
        transport_{pubHandle_}
    {
        loadParams();
        img_queue_ = ImageQueue{params_.max_frame_lag, params_.max_skipped_frames};
        if (params_.flow_algorithm == "lucas_kanade")
        {
            algorithm_ptr_ = std::unique_ptr<FlowAlgorithmInterface>(
                    new LucasKanadeAlgorithm(params_.publish_cv_debug));
        }
        if (params_.flow_algorithm == "farneback")
        {
            algorithm_ptr_ = std::unique_ptr<FlowAlgorithmInterface>(
                    new FarnebackAlgorithm(params_.publish_cv_debug));
        }
        else
        {
            std::stringstream ss{};
            ss << "flow algorithm not recognized: "  << params_.flow_algorithm;
            throw std::runtime_error(ss.str());
        }
        algorithm_ptr_->loadParams(priHandle_);
        raw_sub_ = transport_.subscribe("image", 5,&FlowController::enqueueFrame, this);
        if (params_.publish_cv_debug)
        {
            debug_pub_ = transport_.advertise("debug", 1);
        }
    }

    double FlowController::getRate()
    {
        return params_.rate;
    }

    void FlowController::enqueueFrame(const sensor_msgs::ImageConstPtr& img)
    {
	    ROS_DEBUG_STREAM("Recieved Message time => " << img->header.stamp);
	    img_queue_.pushFrame(img);
    }

    cv_bridge::CvImageConstPtr FlowController::convertToCvPtr(sensor_msgs::ImageConstPtr& img)
    {
       //auto encoding = "bgr8";
        cv_bridge::CvImageConstPtr  cvImg;
        try
        {
            cvImg = cv_bridge::toCvShare(img);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR_STREAM("Could not convert from ");// << img->encoding.c_str() << "to "<< encoding);

        }
        return cvImg;
    }


    void FlowController::process()
    {
	    ROS_DEBUG_STREAM("Processing img_queue_ size: " << img_queue_.size());
        

        if (img_queue_.size())
        {
            auto start = ros::Time::now();
            auto frame = img_queue_.popFrame();
            auto cvImg =  convertToCvPtr(frame);
            algorithm_ptr_->process(cvImg);
            if (params_.publish_cv_debug)
            {
                auto img = algorithm_ptr_->getDebugImage();
                auto img_msg = img.toImageMsg();
                debug_pub_.publish(img_msg);
            }
            auto duration = ros::Time::now() - start;
	        ROS_DEBUG_STREAM("Processed Frame: recieved time => " <<
	                frame->header.stamp << " processing time => " << duration <<
	                " hz => " << 1/duration.toSec());
        }
        else
        {
            ROS_DEBUG("No frames available");
        }
        
    }


    void FlowController::loadParams()
    {
        ROS_INFO("Params:");
        priHandle_.param("rate", params_.rate, 30.0);
        ROS_INFO_STREAM("    rate:               " << params_.rate);
        priHandle_.param("max_frame_lag", params_.max_frame_lag, 10);
        ROS_INFO_STREAM("    max_frame_lag:      " << params_.max_frame_lag);
         priHandle_.param("max_skipped_frames", params_.max_skipped_frames, 3);
        ROS_INFO_STREAM("    max_skipped_frames: " << params_.max_skipped_frames);
         priHandle_.param("publish_cv_debug", params_.publish_cv_debug, false);
        ROS_INFO_STREAM("    publish_cv_debug:   " << params_.publish_cv_debug);
        priHandle_.param("flow_algorithm", params_.flow_algorithm, std::string("lucas_kanade"));
        ROS_INFO_STREAM("    flow_algorithm:     " << params_.flow_algorithm);
    }

}
