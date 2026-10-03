// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IControlLimits related ------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setPosLimits(int axis, double min, double max)
#else
bool AmorControlBoard::setLimits(int axis, double min, double max)
#endif
{
    yCError(ACB) << "setLimits() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getPosLimits(int axis, double * min, double * max)
#else
bool AmorControlBoard::getLimits(int axis, double * min, double * max)
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

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, axis, &parameters) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_joint_info() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    if (parameters.lowerJointLimit == 0.0 && parameters.upperJointLimit == 0.0)
    {
        *min = -180.0;
        *max = 180.0;
    }
    else
    {
        *min = toDeg(parameters.lowerJointLimit);
        *max = toDeg(parameters.upperJointLimit);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setVelLimits(int axis, double min, double max)
#else
bool AmorControlBoard::setVelLimits(int axis, double min, double max)
#endif
{
    yCError(ACB) << "setVelLimits() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getVelLimits(int axis, double * min, double * max)
#else
bool AmorControlBoard::getVelLimits(int axis, double * min, double * max)
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

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, axis, &parameters) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_joint_info() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *max = toDeg(parameters.maxVelocity);
    *min = -(*max);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
