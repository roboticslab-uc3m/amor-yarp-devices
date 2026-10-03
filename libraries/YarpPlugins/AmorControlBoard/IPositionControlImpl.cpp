// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IPositionControl related --------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getAxes(std::size_t & ax)
{
    ax = AMOR_NUM_JOINTS;
    return yarp::dev::ReturnValue_ok;
}
#else
bool AmorControlBoard::getAxes(int * ax)
{
    *ax = AMOR_NUM_JOINTS;
    return true;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::positionMove(int j, double ref)
#else
bool AmorControlBoard::positionMove(int j, double ref)
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
        yCError(ACB) << "amor_get_actual_positions(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    positions[j] = toRad(ref);

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_positions(handle, positions) == AMOR_SUCCESS ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::positionMove(const double * refs)
#else
bool AmorControlBoard::positionMove(const double * refs)
#endif
{
    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] = toRad(refs[j]);
    }

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_positions(handle, positions) == AMOR_SUCCESS ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::positionMove(int n_joint, const int * joints, const double * refs)
#else
bool AmorControlBoard::positionMove(int n_joint, const int * joints, const double * refs)
#endif
{
    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); n_joint < AMOR_NUM_JOINTS && amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_positions(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < n_joint; j++)
    {
        positions[joints[j]] = toRad(refs[j]);
    }

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_positions(handle, positions) == AMOR_SUCCESS ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::relativeMove(int j, double delta)
#else
bool AmorControlBoard::relativeMove(int j, double delta)
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
        yCError(ACB) << "amor_get_actual_positions(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    positions[j] += toRad(delta);

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_positions(handle, positions) == AMOR_SUCCESS ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::relativeMove(const double * deltas)
#else
bool AmorControlBoard::relativeMove(const double * deltas)
#endif
{
    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] += toRad(deltas[j]);
    }

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_positions(handle, positions) == AMOR_SUCCESS ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::relativeMove(int n_joint, const int * joints, const double * deltas)
#else
bool AmorControlBoard::relativeMove(int n_joint, const int * joints, const double * deltas)
#endif
{
    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); n_joint < AMOR_NUM_JOINTS && amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_actual_positions(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < n_joint; j++)
    {
        positions[joints[j]] += toRad(deltas[j]);
    }

    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_set_positions(handle, positions) == AMOR_SUCCESS ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::checkMotionDone(int j, bool & flag)
#else
bool AmorControlBoard::checkMotionDone(int j, bool * flag)
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

    return checkMotionDone(flag);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::checkMotionDone(bool & flag)
#else
bool AmorControlBoard::checkMotionDone(bool * flag)
#endif
{
    amor_movement_status status;

    if (std::lock_guard lock(handleMutex); amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_movement_status(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);
    return yarp::dev::ReturnValue_ok;
#else
    *flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::checkMotionDone(const std::vector<int> & joints, bool & flag)
#else
bool AmorControlBoard::checkMotionDone(int n_joint, const int * joints, bool * flag)
#endif
{
    amor_movement_status status;

    if (std::lock_guard lock(handleMutex); amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_movement_status(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);
    return yarp::dev::ReturnValue_ok;
#else
    *flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setTrajSpeed(int j, double sp)
#else
bool AmorControlBoard::setRefSpeed(int j, double sp)
#endif
{
    yCError(ACB) << "setTrajSpeed() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setTrajSpeeds(const double * spds)
#else
bool AmorControlBoard::setRefSpeeds(const double * spds)
#endif
{
    yCError(ACB) << "setTrajSpeeds() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setTrajSpeeds(int n_joint, const int * joints, const double * spds)
#else
bool AmorControlBoard::setRefSpeeds(int n_joint, const int * joints, const double * spds)
#endif
{
    yCError(ACB) << "setTrajSpeeds() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setTrajAcceleration(int j, double acc)
#else
bool AmorControlBoard::setRefAcceleration(int j, double acc)
#endif
{
    yCError(ACB) << "setTrajAcceleration() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setTrajAccelerations(const double * accs)
#else
bool AmorControlBoard::setRefAccelerations(const double * accs)
#endif
{
    yCError(ACB) << "setTrajAccelerations() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setTrajAccelerations(int n_joint, const int * joints, const double * accs)
#else
bool AmorControlBoard::setRefAccelerations(int n_joint, const int * joints, const double * accs)
#endif
{
    yCError(ACB) << "setTrajAccelerations() not available";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}


// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTrajSpeed(int j, double *ref)
#else
bool AmorControlBoard::getRefSpeed(int j, double *ref)
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

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_joint_info(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *ref = toDeg(parameters.maxVelocity);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTrajSpeeds(double * spds)
#else
bool AmorControlBoard::getRefSpeeds(double * spds)
#endif
{
    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
        {
            yCError(ACB) << "amor_get_joint_info(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return false;
#endif
        }

        spds[j] = toDeg(parameters.maxVelocity);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTrajSpeeds(int n_joint, const int * joints, double * spds)
#else
bool AmorControlBoard::getRefSpeeds(int n_joint, const int * joints, double * spds)
#endif
{
    for (int j = 0; j < n_joint; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, joints[j], &parameters) != AMOR_SUCCESS)
        {
            yCError(ACB) << "amor_get_joint_info(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return false;
#endif
        }

        spds[j] = toDeg(parameters.maxVelocity);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTrajAcceleration(int j, double * acc)
#else
bool AmorControlBoard::getRefAcceleration(int j, double * acc)
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

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_joint_info(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *acc = toDeg(parameters.maxAcceleration);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTrajAccelerations(double * accs)
#else
bool AmorControlBoard::getRefAccelerations(double * accs)
#endif
{
    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
        {
            yCError(ACB) << "amor_get_joint_info(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return false;
#endif
        }

        accs[j] = toDeg(parameters.maxAcceleration);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTrajAccelerations(int n_joint, const int * joints, double * accs)
#else
bool AmorControlBoard::getRefAccelerations(int n_joint, const int * joints, double * accs)
#endif
{
    for (int j = 0; j < n_joint; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, joints[j], &parameters) != AMOR_SUCCESS)
        {
            yCError(ACB) << "amor_get_joint_info(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return false;
#endif
        }

        accs[j] = toDeg(parameters.maxAcceleration);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::stop(int j)
#else
bool AmorControlBoard::stop(int j)
#endif
{
    yCWarning(ACB, "Selective stop not available, stopping all joints at once (%d)", j);

    if (!indexWithinRange(j))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    return stop();
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::stop()
#else
bool AmorControlBoard::stop()
#endif
{
    std::lock_guard lock(handleMutex);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return amor_controlled_stop(handle) == AMOR_SUCCESS ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return amor_controlled_stop(handle) == AMOR_SUCCESS;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::stop(int n_joint, const int * joints)
#else
bool AmorControlBoard::stop(int n_joint, const int * joints)
#endif
{
    yCWarning(ACB, "Selective stop not available, stopping all joints at once (%d)", n_joint);
    return stop();
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTargetPosition(int joint, double * ref)
#else
bool AmorControlBoard::getTargetPosition(int joint, double * ref)
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

    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_req_positions(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    *ref = toDeg(positions[joint]);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTargetPositions(double * refs)
#else
bool AmorControlBoard::getTargetPositions(double * refs)
#endif
{
    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_req_positions(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        refs[j] = toDeg(positions[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getTargetPositions(int n_joint, const int * joints, double * refs)
#else
bool AmorControlBoard::getTargetPositions(int n_joint, const int * joints, double * refs)
#endif
{
    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(handleMutex); amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACB) << "amor_get_req_positions(): " << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    for (int j = 0; j < n_joint; j++)
    {
        refs[j] = toDeg(positions[joints[j]]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
