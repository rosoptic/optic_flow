#ifndef IMAGE_QUEUE_H
#define IMAGE_QUEUE_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <queue>

namespace optic_flow
{
    class ImageQueue
    {
        public:
            ImageQueue();
            ImageQueue(int max_frames, int max_skipped_frames);
            void pushFrame(const sensor_msgs::ImageConstPtr& frame);
            sensor_msgs::ImageConstPtr popFrame();
            int size();
        private:
            int max_frame_lag_;
            int max_skipped_frames_;
            std::queue<sensor_msgs::ImageConstPtr> queue_;
    };
}

#endif
