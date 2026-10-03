// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ IVelocityControl related ----------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::velocityMove(int j, double sp)
#else
bool AmorControlBoard::velocityMove(int j, double sp)
#endif
{
    if (!indexWithinRange(j))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_velocities() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    velocities[j] = toRad(sp);

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS
        ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::velocityMove(const double * sp)
#else
bool AmorControlBoard::velocityMove(const double * sp)
#endif
{
    AMOR_VECTOR7 velocities;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        velocities[j] = toRad(sp[j]);
    }

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS
        ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
#endif
}

// ----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::velocityMove(int n_joint, const int * joints, const double * spds)
#else
bool AmorControlBoard::velocityMove(int n_joint, const int * joints, const double * spds)
#endif
{
    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); n_joint < AMOR_NUM_JOINTS && amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_velocities() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < n_joint; j++)
    {
        velocities[joints[j]] = toRad(spds[j]);
    }

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS
        ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTargetVelocity(int joint, double * vel)
#else
bool AmorControlBoard::getRefVelocity(int joint, double * vel)
#endif
{
    if (!indexWithinRange(joint))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_req_velocities() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *vel = toDeg(velocities[joint]);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTargetVelocities(double * vels)
#else
bool AmorControlBoard::getRefVelocities(double * vels)
#endif
{
    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_req_velocities() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        vels[j] = toDeg(velocities[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTargetVelocities(int n_joint, const int * joints, double * vels)
#else
bool AmorControlBoard::getRefVelocities(int n_joint, const int * joints, double * vels)
#endif
{
    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_req_velocities() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < n_joint; j++)
    {
        vels[j] = toDeg(velocities[joints[j]]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
