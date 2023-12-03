// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IAxisInfo related ------------------------------------

bool AmorControlBoard::getAxisName(int axis, std::string& name)
{
    yCTrace(ACB, "%d", axis);

    if (!indexWithinRange(axis))
    {
        return false;
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
            yCError(ACB, "Unrecognized axis: %d", axis);
            return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    yCTrace(ACB, "%d", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;

    return true;
}

// -----------------------------------------------------------------------------
