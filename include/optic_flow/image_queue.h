#ifndef IMAGE_QUEUE_H
#define IMAGE_QUEUE_H

namespace optic_flow
{
    class ImageQueue
    {
        public:
            void push();
            void pop();
            int size();
        private:
            int max_frames_;
            int max_skipped_frames_;
            int skipped_frames_;
    };
}

#endif
