// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __AMOR_CARTESIAN_CONTROL_HPP__
#define __AMOR_CARTESIAN_CONTROL_HPP__

#include <mutex>
#include <vector>

#include <amor.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"
#include "ICartesianSolver.h"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup AmorCartesianControl
 * @brief Contains roboticslab::AmorCartesianControl.
 */

/**
 * @ingroup AmorCartesianControl
 * @brief The AmorCartesianControl class implements ICartesianControl.
 *
 * Uses the roll-pitch-yaw (RPY) angle representation.
 */
class AmorCartesianControl : public yarp::dev::DeviceDriver,
                             public ICartesianControl
{
public:
    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --
    yarp::dev::ReturnValue getState(ControllerState & state) override;
    yarp::dev::ReturnValue solvePose(const std::vector<double> & xd, std::vector<double> & q) override;
    yarp::dev::ReturnValue moveJoint(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveLinear(const std::vector<double> & xd) override;
    yarp::dev::ReturnValue moveVelocity(const std::vector<double> & xdotd) override;
    yarp::dev::ReturnValue gravityCompensation() override;
    yarp::dev::ReturnValue forceControl(const std::vector<double> & fd) override;
    yarp::dev::ReturnValue stopControl() override;
    yarp::dev::ReturnValue changeTool(const std::vector<double> & x) override;
    yarp::dev::ReturnValue actuateTool(Actuator command) override;
    void pose(const std::vector<double> & x) override;
    void twist(const std::vector<double> & xdot) override;
    void wrench(const std::vector<double> &w) override;
    yarp::dev::ReturnValue setParameter(Config vocab, config_value_t value) override;
    yarp::dev::ReturnValue getParameter(Config vocab, config_value_t & value) override;
    yarp::dev::ReturnValue setParameters(const config_map_t & params) override;
    yarp::dev::ReturnValue getParameters(config_map_t & params) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    bool checkJointVelocities(const std::vector<double> & qdot);

    AMOR_HANDLE handle {AMOR_INVALID_HANDLE};
    bool ownsHandle {true};
    mutable std::mutex * handleMutex {nullptr};

    yarp::dev::PolyDriver cartesianDevice;
    ICartesianSolver * iCartesianSolver {nullptr};

    ICartesianControl::Mode currentState {Mode::NONE};
    double gain {0.0};

    std::vector<double> qdotMax;

    ICartesianSolver::Frame referenceFrame;
};

} // namespace roboticslab

#endif // __AMOR_CARTESIAN_CONTROL_HPP__
