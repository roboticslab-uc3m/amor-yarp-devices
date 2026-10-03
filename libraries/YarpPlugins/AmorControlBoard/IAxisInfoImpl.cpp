// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IAxisInfo related ------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getAxisName(int axis, std::string & name)
#else
bool AmorControlBoard::getAxisName(int axis, std::string & name)
#endif
{
    if (!indexWithinRange(axis))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
    return false;
#endif
    }

    switch (axis)
    {
        case 0:
            name = "A1";
            break;
        case 1:
            name = "A2";
            break;
        case 2:
            name = "A2.5";
            break;
        case 3:
            name = "A3";
            break;
        case 4:
            name = "A4";
            break;
        case 5:
            name = "A5";
            break;
        case 6:
            name = "A6";
            break;
        default:
            yCError(ACB) << "Unrecognized axis:" << axis;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
            return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getJointType(int axis, yarp::dev::JointTypeEnum & type)
#else
bool AmorControlBoard::getJointType(int axis, yarp::dev::JointTypeEnum & type)
#endif
{
    if (!indexWithinRange(axis))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
    return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
#else
    type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
#endif

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
