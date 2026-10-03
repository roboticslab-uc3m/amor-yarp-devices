// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include "KinematicRepresentation.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- ICartesianControl Related ------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::getState(ControllerState & state)
{
    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(*handleMutex); amor_get_cartesian_position(handle, positions) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_get_cartesian_position() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    state.x.resize(6);

    state.x[0] = positions[0] * 0.001; // [m]
    state.x[1] = positions[1] * 0.001;
    state.x[2] = positions[2] * 0.001;

    state.x[3] = positions[3]; // [rad]
    state.x[4] = positions[4];
    state.x[5] = positions[5];

    KinRepresentation::encodePose(state.x, state.x, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::RPY);

    state.mode = currentState;
    state.timestamp = yarp::os::Time::now();

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::solvePose(const std::vector<double> & xd, std::vector<double> & q)
{
    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(*handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_get_actual_positions() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS);

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    if (!iCartesianSolver->inverseKinematics(xd, currentQ, q, referenceFrame))
    {
        yCError(ACC) << "invKin() failed";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::moveJoint(const std::vector<double> & xd)
{
    std::vector<double> qd;

    if (!solvePose(xd, qd))
    {
        yCError(ACC) << "solvePose() failed";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    AMOR_VECTOR7 positions;

    for (int i = 0; i < qd.size(); i++)
    {
        positions[i] = KinRepresentation::degToRad(qd[i]);
    }

    if (std::lock_guard lock(*handleMutex); amor_set_positions(handle, positions) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_set_positions() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    currentState = Mode::MOVEJ;

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::moveLinear(const std::vector<double> & xd)
{
    std::vector<double> xd_obj;

    if (referenceFrame == ICartesianSolver::Frame::TCP)
    {
        AMOR_VECTOR7 positions;

        if (std::lock_guard lock(*handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
        {
            yCError(ACC) << "amor_get_actual_positions() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }

        std::vector<double> currentQ(AMOR_NUM_JOINTS);

        for (int i = 0; i < AMOR_NUM_JOINTS; i++)
        {
            currentQ[i] = KinRepresentation::radToDeg(positions[i]);
        }

        std::vector<double> x_base_tcp;

        if (!iCartesianSolver->forwardKinematics(currentQ, x_base_tcp))
        {
            yCError(ACC) << "forwardKinematics() failed";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }

        if (!iCartesianSolver->changeOrigin(xd, x_base_tcp, xd_obj))
        {
            yCError(ACC) << "changeOrigin() failed";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
    }
    else
    {
        xd_obj = xd;
    }

    std::vector<double> xd_rpy;

    KinRepresentation::decodePose(xd_obj, xd_rpy, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::RPY);

    AMOR_VECTOR7 positions;

    positions[0] = xd_rpy[0] * 1000; // [mm]
    positions[1] = xd_rpy[1] * 1000;
    positions[2] = xd_rpy[2] * 1000;

    positions[3] = xd_rpy[3]; // [rad]
    positions[4] = xd_rpy[4];
    positions[5] = xd_rpy[5];

    if (std::lock_guard lock(*handleMutex); amor_set_cartesian_positions(handle, positions) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_set_cartesian_positions() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    currentState = Mode::MOVEL;

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::moveVelocity(const std::vector<double> & xdotd)
{
    if (referenceFrame == ICartesianSolver::Frame::TCP)
    {
        yCWarning(ACC) << "TCP frame not supported yet in movv command";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_not_ready;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#endif
    }

    ControllerState state;

    if (!getState(state))
    {
        yCError(ACC) << "getState() failed";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    std::vector<double> xdotd_rpy;

    KinRepresentation::decodeVelocity(state.x, xdotd, xdotd_rpy, KinRepresentation::coordinate_system::CARTESIAN, KinRepresentation::orientation_system::RPY);

    AMOR_VECTOR7 velocities;

    velocities[0] = xdotd_rpy[0] * 1000; // [mm/s]
    velocities[1] = xdotd_rpy[1] * 1000;
    velocities[2] = xdotd_rpy[2] * 1000;

    // FIXME: un-shuffle coordinates
    velocities[3] = xdotd_rpy[4]; // [rad/s]
    velocities[4] = -xdotd_rpy[5];
    velocities[5] = xdotd_rpy[3];

    if (std::lock_guard lock(*handleMutex); amor_set_cartesian_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_set_cartesian_velocities() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    currentState = Mode::MOVEV;

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::gravityCompensation()
{
    yCWarning(ACC) << "gravityCompensation() not implemented";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#endif
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::forceControl(const std::vector<double> & fd)
{
    yCWarning(ACC) << "forceControl() not implemented";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#endif
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::stopControl()
{
    currentState = Mode::NONE;

    if (std::lock_guard lock(*handleMutex); amor_controlled_stop(handle) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_controlled_stop() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::changeTool(const std::vector<double> & x)
{
    yCWarning(ACC) << "changeTool() not supported on ACC";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#endif
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::actuateTool(Actuator command)
{
    AMOR_RESULT (*amor_command)(AMOR_HANDLE);

    switch (command)
    {
    case Actuator::CLOSE:
        amor_command = amor_close_hand;
        break;
    case Actuator::OPEN:
        amor_command = amor_open_hand;
        break;
    case Actuator::STOP:
        amor_command = amor_stop_hand;
        break;
    default:
        yCError(ACC) << "Unrecognized act() command" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(command));
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    if (std::lock_guard lock(*handleMutex); amor_command(handle) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_command() failed:" << amor_error();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

void AmorCartesianControl::pose(const std::vector<double> & x)
{
    yCWarning(ACC) << "pose() not supported, falling back to moveJoint()";
    moveJoint(x);
}

// -----------------------------------------------------------------------------

void AmorCartesianControl::twist(const std::vector<double> & xdot)
{
    AMOR_VECTOR7 positions;

    if (std::lock_guard lock(*handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_get_actual_positions() failed:" << amor_error();
        return;
    }

    std::vector<double> currentQ(AMOR_NUM_JOINTS), qdot;

    for (int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        currentQ[i] = KinRepresentation::radToDeg(positions[i]);
    }

    if (!iCartesianSolver->diffInverseKinematics(currentQ, xdot, qdot, referenceFrame))
    {
        yCError(ACC) << "diffInvKin() failed";
        return;
    }

    if (!checkJointVelocities(qdot))
    {
        std::lock_guard lock(*handleMutex);
        amor_controlled_stop(handle);
        return;
    }

    AMOR_VECTOR7 velocities;

    for (int i = 0; i < qdot.size(); i++)
    {
        velocities[i] = KinRepresentation::degToRad(qdot[i]);
    }

    if (std::lock_guard lock(*handleMutex); amor_set_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        yCError(ACC) << "amor_set_velocities() failed:" << amor_error();
        return;
    }
}

// -----------------------------------------------------------------------------

void AmorCartesianControl::wrench(const std::vector<double> & w)
{
    yCWarning(ACC) << "wrench() not supported";
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::setParameter(Config vocab, config_value_t value)
{
    if (currentState != Mode::NONE)
    {
        yCError(ACC) << "Unable to set config parameter while controlling";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_not_ready;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#endif
    }

    switch (vocab)
    {
    case Config::GAIN:
        if (std::get<double>(value) < 0.0)
        {
            yCError(ACC) << "Controller gain cannot be negative";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
        gain = std::get<double>(value);
        break;
    case Config::FRAME:
        if (std::get<yarp::conf::vocab32_t>(value) != static_cast<yarp::conf::vocab32_t>(ICartesianSolver::Frame::BASE) &&
            std::get<yarp::conf::vocab32_t>(value) != static_cast<yarp::conf::vocab32_t>(ICartesianSolver::Frame::TCP))
        {
            yCError(ACC) << "Unrecognized or unsupported reference frame vocab";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
        }
        referenceFrame = static_cast<ICartesianSolver::Frame>(std::get<yarp::conf::vocab32_t>(value));
        break;
    default:
        yCError(ACC) << "Unrecognized or unsupported config parameter key:"
                     << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::getParameter(Config vocab, config_value_t & value)
{
    switch (vocab)
    {
    case Config::GAIN:
        value = gain;
        break;
    case Config::FRAME:
        value = static_cast<yarp::conf::vocab32_t>(referenceFrame);
        break;
    default:
        yCError(ACC) << "Unrecognized or unsupported config parameter key:"
                     << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(vocab));
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::setParameters(const config_map_t & params)
{
    if (currentState != Mode::NONE)
    {
        yCError(ACC) << "Unable to set config parameters while controlling";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_not_ready;
#else
        return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#endif
    }

    bool ok = true;

    for (const auto & [vocab, value] : params)
    {
        ok &= setParameter(vocab, value);
    }

    return ok ? yarp::dev::ReturnValue_ok
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
              : yarp::dev::ReturnValue_error_method_failed;
#else
              : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#endif
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue AmorCartesianControl::getParameters(config_map_t & params)
{
    params.emplace(Config::GAIN, gain);
    params.emplace(Config::FRAME, static_cast<yarp::conf::vocab32_t>(referenceFrame));
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
