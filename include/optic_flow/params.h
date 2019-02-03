#ifndef PARAMS_H
#define PARAMS_H
#include <ros/ros.h>

namespace optic_flow
{
    struct OpticFlowParams
    {
        int rate;
    };

    OpticFlowParams getParams();
}

#endif
