// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <algorithm>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ ICurrentControl Related -----------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getNumberOfMotors(int * ax)
{
    std::size_t axes;
    auto ret = getAxes(axes);
    *ax = static_cast<int>(axes);
    return ret;
}
#else
bool AmorControlBoard::getNumberOfMotors(int * ax)
{
    return getAxes(ax);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getCurrent(int m, double * curr)
#else
bool AmorControlBoard::getCurrent(int m, double * curr)
#endif
{
    if (!indexWithinRange(m))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_currents() failed:", amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *curr = currents[m];

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getCurrents(double * currs)
#else
bool AmorControlBoard::getCurrents(double * currs)
#endif
{
    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_currents() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    std::copy(currents, currents + AMOR_NUM_JOINTS, currs);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getCurrentRange(int m, double * min, double * max)
#else
bool AmorControlBoard::getCurrentRange(int m, double * min, double * max)
#endif
{
    if (!indexWithinRange(m))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, m, &parameters) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_joint_info() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *min = -parameters.maxCurrent;
    *max = parameters.maxCurrent;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getCurrentRanges(double * min, double * max)
#else
bool AmorControlBoard::getCurrentRanges(double * min, double * max)
#endif
{
    bool ok = true;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        ok &= getCurrentRange(j, &min[j], &max[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setRefCurrent(int m, double curr)
#else
bool AmorControlBoard::setRefCurrent(int m, double curr)
#endif
{
    if (!indexWithinRange(m))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_currents() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    currents[m] = curr;

    if (std::lock_guard lock(handleMutex); amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_set_currents() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
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
yarp::dev::ReturnValue AmorControlBoard::setRefCurrents(const double * currs)
#else
bool AmorControlBoard::setRefCurrents(const double * currs)
#endif
{
    AMOR_VECTOR7 currents;

    std::copy(currs, currs + AMOR_NUM_JOINTS, currents);

    if (std::lock_guard lock(handleMutex); amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_set_currents() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
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
yarp::dev::ReturnValue AmorControlBoard::setRefCurrents(int n_motor, const int * motors, const double * currs)
#else
bool AmorControlBoard::setRefCurrents(int n_motor, const int * motors, const double * currs)
#endif
{
    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_actual_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_currents() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int i = 0; i < n_motor; i++)
    {
        currents[motors[i]] = currs[i];
    }

    if (std::lock_guard lock(handleMutex); amor_set_currents(handle, currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_set_currents() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
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
yarp::dev::ReturnValue AmorControlBoard::getRefCurrent(int m, double *curr)
#else
bool AmorControlBoard::getRefCurrent(int m, double *curr)
#endif
{
    if (!indexWithinRange(m))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_req_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_req_currents() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *curr = currents[m];

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getRefCurrents(double * currs)
#else
bool AmorControlBoard::getRefCurrents(double * currs)
#endif
{
    AMOR_VECTOR7 currents;

    if (std::lock_guard lock(handleMutex); amor_get_req_currents(handle, &currents) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_req_currents() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    std::copy(currents, currents + AMOR_NUM_JOINTS, currs);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
