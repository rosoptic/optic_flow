#include <image_queue.h>

namespace optic_flow
{
    ImageQueue::ImageQueue(): ImageQueue{5,3} {}
    ImageQueue::ImageQueue(int max_frames, int max_skipped_frames):
        max_frame_lag_{max_frames},
        max_skipped_frames_{max_skipped_frames},
        queue_{} 
    {
        if (!max_frame_lag_ > 0)
            throw std::invalid_argument("max_frames, must be > 0");
        if (!max_skipped_frames_ > 0)
            throw std::invalid_argument("max_skipped)frames, must be > 0");
    }

    void ImageQueue::pushFrame(const sensor_msgs::ImageConstPtr& frame)
    {
	    ROS_DEBUG_STREAM("Storing Frame: time => " << frame->header.stamp);
	    queue_.push(frame);
    }

    sensor_msgs::ImageConstPtr ImageQueue::popFrame()
    {
        if (!size())
            throw std::runtime_error("Cannot pop frame when queue is empty");

        auto frame = queue_.front();
        if (size() > max_frame_lag_)
        {
            ROS_DEBUG("The image queue is at maximum size, dropping frames");
            auto skipped_frames = 0;
            while (size() > max_frame_lag_)
            {
                if (++skipped_frames > max_skipped_frames_ )
                {
                    ROS_WARN("Cannot skip more frames, sensor lag occurring");
                    break;
                }
                frame = queue_.front();
                ROS_DEBUG_STREAM("Dropping Frame: " << frame->header.stamp);
                queue_.pop();
            }
        }
        frame = queue_.front();
        ROS_DEBUG_STREAM("Popping Frame: " << frame->header.stamp);
        queue_.pop();
        return frame;
    }

    int ImageQueue::size()
    {
        return queue_.size();
    }
}

