#include <flow_controller.h>

namespace optic_flow 
{
    FlowController::FlowController():img_queue_{}
    {
        ROS_INFO("Constructing FlowController");
    }

    FlowController::~FlowController()
    {
        ROS_INFO("Destructing FlowController");
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
	        ROS_DEBUG_STREAM("Popping Message time => " << frame->header.stamp);
            img_queue_.pop();

        }
        
    }

}
