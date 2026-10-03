// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <algorithm> // std::fill_n

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ IEncoders related -----------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::resetEncoder(int j)
#else
bool AmorControlBoard::resetEncoder(int j)
#endif
{
    yCError(ACB) << "resetEncoder() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::resetEncoders()
#else
bool AmorControlBoard::resetEncoders()
#endif
{
    yCError(ACB) << "resetEncoders() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setEncoder(int j, double val)
#else
bool AmorControlBoard::setEncoder(int j, double val)
#endif
{
    yCError(ACB) << "setEncoder() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setEncoders(const double * vals)
#else
bool AmorControlBoard::setEncoders(const double * vals)
#endif
{
    yCError(ACB) << "setEncoders() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getEncoder(int j, double * v)
#else
bool AmorControlBoard::getEncoder(int j, double * v)
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

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_positions() failed: " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *v = toDeg(positions[j]);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getEncoders(double * encs)
#else
bool AmorControlBoard::getEncoders(double * encs)
#endif
{
    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_positions() failed: " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        encs[j] = toDeg(positions[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getEncoderSpeed(int j, double * sp)
#else
bool AmorControlBoard::getEncoderSpeed(int j, double * sp)
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
        yCError(ACB) << "amor_get_actual_velocities() failed: " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *sp = toDeg(velocities[j]);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getEncoderSpeeds(double * spds)
#else
bool AmorControlBoard::getEncoderSpeeds(double * spds)
#endif
{
    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_velocities() failed: " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        spds[j] = toDeg(velocities[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getEncoderAcceleration(int j, double * spds)
#else
bool AmorControlBoard::getEncoderAcceleration(int j, double * spds)
#endif
{
    //yCError(ACB) << "getEncoderAcceleration() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getEncoderAccelerations(double * accs)
#else
bool AmorControlBoard::getEncoderAccelerations(double * accs)
#endif
{
    //yCError(ACB) << "getEncoderAccelerations() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// ------------------ IEncodersTimed related -----------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getEncodersTimed(double * encs, double * time)
#else
bool AmorControlBoard::getEncodersTimed(double * encs, double * time)
#endif
{
    double now = yarp::os::Time::now();
    std::fill_n(time, AMOR_NUM_JOINTS, now);
    return getEncoders(encs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getEncoderTimed(int j, double * encs, double * time)
#else
bool AmorControlBoard::getEncoderTimed(int j, double * encs, double * time)
#endif
{
    *time = yarp::os::Time::now();
    return getEncoder(j, encs);
}

// -----------------------------------------------------------------------------
