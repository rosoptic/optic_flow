#include <params.h>

namespace optic_flow
{
    OpticFlowParams getParams()    
    {
        OpticFlowParams params{};
        ros::param::param<int>("~rate", params.rate, 30);
        ROS_INFO_STREAM("~rate: " << params.rate);
        return params;
    }

}

