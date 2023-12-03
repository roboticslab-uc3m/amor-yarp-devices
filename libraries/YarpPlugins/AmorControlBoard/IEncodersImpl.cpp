// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ IEncoders related -----------------------------------------

bool AmorControlBoard::resetEncoder(int j)
{
    yCError(ACB, "resetEncoder() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::resetEncoders()
{
    yCError(ACB, "resetEncoders() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setEncoder(int j, double val)
{
    yCError(ACB, "setEncoder() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setEncoders(const double *vals)
{
    yCError(ACB, "setEncoders() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getEncoder(int j, double *v)
{
    yCTrace(ACB, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB, "amor_get_actual_positions() failed: %s", amor_error());
        return false;
    }

    *v = toDeg(positions[j]);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getEncoders(double *encs)
{
    yCTrace(ACB, "");

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB, "amor_get_actual_positions() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        encs[j] = toDeg(positions[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getEncoderSpeed(int j, double *sp)
{
    yCTrace(ACB, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(ACB, "amor_get_actual_velocities() failed: %s", amor_error());
        return false;
    }

    *sp = toDeg(velocities[j]);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getEncoderSpeeds(double *spds)
{
    yCTrace(ACB, "");

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(ACB, "amor_get_actual_velocities() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        spds[j] = toDeg(velocities[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getEncoderAcceleration(int j, double *spds)
{
    //yCError(ACB, "getEncoderAcceleration() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getEncoderAccelerations(double *accs)
{
    //yCError(ACB, "getEncoderAccelerations() not available");
    return false;
}

// ------------------ IEncodersTimed related -----------------------------------------

bool AmorControlBoard::getEncodersTimed(double *encs, double *time)
{
    yCTrace(ACB, "");
    double now = yarp::os::Time::now();
    std::fill_n(time, AMOR_NUM_JOINTS, now);
    return getEncoders(encs);
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getEncoderTimed(int j, double *encs, double *time)
{
    yCTrace(ACB, "%d", j);
    *time = yarp::os::Time::now();
    return getEncoder(j, encs);
}

// -----------------------------------------------------------------------------
