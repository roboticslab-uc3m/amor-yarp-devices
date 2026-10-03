// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool AmorControlBoard::indexWithinRange(int idx)
{
    if (idx < 0 || idx >= AMOR_NUM_JOINTS)
    {
        yCError(ACB, "Index out of range: < 0 || %d >= %d", idx, AMOR_NUM_JOINTS);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

double AmorControlBoard::toDeg(double rad)
{
    return rad * 180 / M_PI;
}

// -----------------------------------------------------------------------------

double AmorControlBoard::toRad(double deg)
{
    return deg * M_PI / 180;
}

// -----------------------------------------------------------------------------
