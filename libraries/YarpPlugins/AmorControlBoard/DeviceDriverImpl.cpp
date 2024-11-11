// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <vector>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_CAN_LIBRARY = "libeddriver.so";
constexpr auto DEFAULT_CAN_PORT = 0;

// ------------------- DeviceDriver related ------------------------------------

bool AmorControlBoard::open(yarp::os::Searchable& config)
{
    int major, minor, build;
    amor_get_library_version(&major, &minor, &build);

    yCInfo(ACB, "AMOR API library version %d.%d.%d", major, minor, build);
    yCInfo(ACB) << "Trying to connect to AMOR...";

    handle = amor_connect((char *)DEFAULT_CAN_LIBRARY, DEFAULT_CAN_PORT);

    if (handle == AMOR_INVALID_HANDLE)
    {
        yCError(ACB) << "Could not get AMOR handle:" << amor_error();
        return false;
    }

    yCInfo(ACB) << "Acquired AMOR handle!";

    AMOR_JOINT_INFO jointInfo[AMOR_NUM_JOINTS];
    int jointStatus[AMOR_NUM_JOINTS];

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        if (amor_get_joint_info(handle, j, &jointInfo[j]) != AMOR_SUCCESS)
        {
            yCError(ACB) << "amor_get_joint_info() failed for joint" << j << "with error:" << amor_error();
            return false;
        }

        if (amor_get_status(handle, j, &jointStatus[j]) != AMOR_SUCCESS)
        {
            yCError(ACB) << "amor_get_status() failed for joint" << j << "with error:" << amor_error();
            return false;
        }
    }

    std::vector<double> positions;

    if (!getEncoders(positions.data()))
    {
        yCError(ACB) << "getEncoders() failed";
        return false;
    }

    yCInfo(ACB) << "Current positions (degrees):" << positions;

    // Set position mode.
    if (!positionMove(positions.data()))
    {
        yCError(ACB) << "positionMove() failed";
        return false;
    }

    yarp::os::Value * cartesianControllerName;

    if (config.check("cartesianControllerName", cartesianControllerName, "cartesian controller port"))
    {
        yCInfo(ACB) << "Using AMOR cartesian controller device";

        usingCartesianController = true;

        std::string subdevice = "AmorCartesianControl";
        yarp::os::Value vHandle(&handle, sizeof(handle));
        yarp::os::Value vHandleMutex(&handleMutex, sizeof(handleMutex));
        yarp::os::Property cartesianControllerOptions;

        cartesianControllerOptions.fromString((config.toString()));
        cartesianControllerOptions.put("device", "CartesianControlServer");
        cartesianControllerOptions.put("subdevice", subdevice);
        cartesianControllerOptions.put("name", cartesianControllerName->asString());
        cartesianControllerOptions.put("handle", vHandle);
        cartesianControllerOptions.put("handleMutex", vHandleMutex);

        cartesianControllerDevice.open(cartesianControllerOptions);

        if (!cartesianControllerDevice.isValid())
        {
            yCError(ACB) << "AMOR cartesian controller device not valid";
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::close()
{
    if (usingCartesianController)
    {
        cartesianControllerDevice.close();
    }

    if (handle != AMOR_INVALID_HANDLE)
    {
        amor_emergency_stop(handle);
        amor_release(handle);

        handle = AMOR_INVALID_HANDLE;
    }

    return true;
}

// -----------------------------------------------------------------------------
