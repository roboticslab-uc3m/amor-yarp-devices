// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IControlMode related ------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    if (!indexWithinRange(j))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    avail = {yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION};
    return yarp::dev::ReturnValue_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
#else
bool AmorControlBoard::getControlMode(int j, int * mode)
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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    mode = static_cast<yarp::dev::ControlModeEnum>(controlMode);
    return yarp::dev::ReturnValue_ok;
#else
    *mode = controlMode;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool AmorControlBoard::getControlModes(int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    modes.resize(AMOR_NUM_JOINTS);
#endif

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getControlMode(i, modes[i]);
#else
        ok &= getControlMode(i, &modes[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool AmorControlBoard::getControlModes(int n_joint, const int * joints, int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    modes.resize(joints.size());
#endif

    #if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    for (unsigned int i = 0; i < joints.size(); i++)
#else
    for (unsigned int i = 0; i < n_joint; i++)
#endif
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getControlMode(joints[i], modes[i]);
#else
        ok &= getControlMode(joints[i], &modes[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setControlMode(int j, yarp::dev::SelectableControlModeEnum mode)
#else
bool AmorControlBoard::setControlMode(int j, int mode)
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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    controlMode = static_cast<yarp::conf::vocab32_t>(mode);
    return yarp::dev::ReturnValue_ok;
#else
    controlMode = mode;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool AmorControlBoard::setControlModes(int n_joint, const int * joints, int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    for (unsigned int i = 0; i < joints.size(); i++)
#else
    for (unsigned int i = 0; i < n_joint; i++)
#endif
    {
        ok &= setControlMode(joints[i], modes[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue AmorControlBoard::setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool AmorControlBoard::setControlModes(int * modes)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------
