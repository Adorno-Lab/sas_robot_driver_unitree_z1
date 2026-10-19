#include "DriverUnitreeZ1.hpp"
#include "unitree_arm_sdk/control/unitreeArm.h"
#include <marinholab/sas/core/sas_clock.hpp>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <stdexcept>

namespace
{
// Upper bound for the blocking moves to the initial and home configurations. With the default gains
// these moves take a few seconds; the bound only protects against a move that never converges.
constexpr double kBlockingMoveTimeoutSec = 60.0;
// Number of samples used to read the robot state before starting a numerical integration.
constexpr int kStateSamplesBeforeIntegration = 50;
// Convergence gain of the QP used in MODE::PositionControl.
constexpr double kPositionControlGain = 5.0;
constexpr Eigen::Index kNumArmJoints = 6;
// Motor error flags (LowlevelState::errorstate) that stop the driver: phase current is too large (0x01),
// phase leakage (0x02), and motor winding overheat (0x04).
constexpr uint8_t kMotorFaultMask = 0x01 | 0x02 | 0x04;
// Motor error flag that the SDK documents as "Ignore".
constexpr uint8_t kMotorIgnoredErrorMask = 0x40;
// Number of consecutive state updates with a motor communication error (LowlevelState::isMotorConnected)
// before it is treated as a fault. The SDK reports each error once, so isolated errors are only logged.
constexpr int kMaxConsecutiveMotorConnectionErrors = 50;

std::string to_hex(const uint8_t& value)
{
    char buffer[8];
    std::snprintf(buffer, sizeof(buffer), "0x%02x", static_cast<unsigned int>(value));
    return std::string(buffer);
}
}

class DriverUnitreeZ1::Impl
{
public:
    std::shared_ptr<UNITREE_ARM::unitreeArm> arm_;
    std::shared_ptr<UNITREE_ARM::CtrlComponents> ctrlComp_;
    Impl()
    {

    };
};


/**
 * @brief DriverUnitreeZ1::DriverUnitreeZ1 constructor of the class.
 * @param shutdown_signaler Use this signaler to break internal loops. This is useful to stop the robot using a signal
 *              interruption by the user.
 * @param mode The operation mode. Select the control strategy to command the robot.
 * @param gripper_attached Use true (default) if the robot is equipped with the Unitree Gripper. Use false otherwise.
 * @param verbosity. Use true (default) to display more information in the terminal.
 * @param open_loop_joint_control_gain The gain used to move the robot towards the initial and home configuration.
 *
 * Example:
 *          // This class follows the SmartArmStack driver principles, in which four methods are required to
 *             start and finish the robot communication.
 *
 *          auto shutdown_signaler = std::make_shared<marinholab::sas::core::ShutdownSignaler>(&kill_this_process);
 *          DriverUnitreeZ1 Z1(shutdown_signaler, DriverUnitreeZ1::MODE::PositionControl);
 *          Z1.connect();    // First method to be called.
 *          Z1.initialize(); // Second method to be called. It is required to connect before to initialize.
 *
 *          // Do fun stuff with the robot here
 *          auto q = Z1.get_joint_positions();
 *
 *          Z1.deinitialize();   // Third method to be called.
 *          Z1.disconnect();     // Fourth method to be called. It is required to deinitialize before to disconnect.
 *
 */
DriverUnitreeZ1::DriverUnitreeZ1(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
                                 const MODE &mode,
                                 const bool &gripper_attached,
                                 const bool& verbosity,
                                 const double&  open_loop_joint_control_gain)
    :shutdown_signaler_{shutdown_signaler}, mode_{mode}, open_loop_joint_control_gain_{open_loop_joint_control_gain},
    verbosity_{verbosity}, gripper_attached_{gripper_attached}
{
    if (!shutdown_signaler_)
        throw std::invalid_argument(std::string(__func__) + ": the shutdown signaler must not be null.");

    impl_       = std::make_shared<DriverUnitreeZ1::Impl>();
    target_joint_velocities_ = VectorXd::Zero(kNumArmJoints);
    // This constructor offers more customized options.
    /*
    impl_->ctrlComp_    = std::make_shared<UNITREE_ARM::CtrlComponents>(0.004, gripper_attached);
    impl_->ctrlComp_->dt = 0.004;//500HZ                               //8071      //8072                                                //500000
    impl_->ctrlComp_->udp = new UNITREE_ARM::UDPPort("192.168.123.220", 8071, 8072);
    impl_->ctrlComp_->armModel = new UNITREE_ARM::Z1Model();// no UnitreeGripper
    impl_->ctrlComp_->armModel->addLoad(0.03);// add 0.03kg payload to the end joint

    impl_->arm_ = std::make_shared<UNITREE_ARM::unitreeArm>(impl_->ctrlComp_.get());// new unitreeArm(ctrlComp1);
    */

    impl_->arm_ = std::make_shared<UNITREE_ARM::unitreeArm>(gripper_attached);


    current_status_ = STATUS::IDLE;
    status_msg_ = std::string("Idle.");

    H_  = MatrixXd::Identity(kNumArmJoints, kNumArmJoints);
    const MatrixXd I = MatrixXd::Identity(kNumArmJoints, kNumArmJoints);
    A_ = MatrixXd(4*kNumArmJoints, kNumArmJoints);
    A_ << -I, I, -I, I;
    solver_ = std::make_unique<DQ_QPOASESSolver>();

    _set_driver_mode(mode);
}

/**
 * @brief DriverUnitreeZ1::DriverUnitreeZ1 deprecated constructor that uses a raw pointer as the shutdown signal.
 *          The pointer is wrapped in a ShutdownSignaler, so the external flag and the signaler stay in sync.
 * @param st_break_loops The external flag used to break the internal loops.
 * @param mode The operation mode. Select the control strategy to command the robot.
 * @param gripper_attached Use true (default) if the robot is equipped with the Unitree Gripper. Use false otherwise.
 * @param verbosity. Use true (default) to display more information in the terminal.
 * @param open_loop_joint_control_gain The gain used to move the robot towards the initial and home configuration.
 */
DriverUnitreeZ1::DriverUnitreeZ1(std::atomic_bool *st_break_loops,
                                 const MODE &mode,
                                 const bool &gripper_attached,
                                 const bool &verbosity,
                                 const double &open_loop_joint_control_gain)
    :DriverUnitreeZ1(std::make_shared<marinholab::sas::core::ShutdownSignaler>(st_break_loops),
                     mode, gripper_attached, verbosity, open_loop_joint_control_gain)
{

}

/**
 * @brief DriverUnitreeZ1::~DriverUnitreeZ1 stops the driver threads before the members they use are destroyed.
 *              It does not move the robot. Call deinitialize() and disconnect() for a proper shutdown.
 */
DriverUnitreeZ1::~DriverUnitreeZ1()
{
    _stop_control_thread();
    _stop_state_thread();
}

/**
 * @brief DriverUnitreeZ1::set_joint_limits_with_gripper sets the joint limits used to clamp every command sent to the
 *              robot. The arm limits are also used by the QP in PositionControl and in the moves to the initial and
 *              home configurations. This method must be called before initialize().
 * @param q_min_with_gripper_rad A vector 7x1 containing the lower joint limits and the lower gripper limit, in radians.
 * @param q_max_with_gripper_rad A vector 7x1 containing the upper joint limits and the upper gripper limit, in radians.
 */
void DriverUnitreeZ1::set_joint_limits_with_gripper(const VectorXd &q_min_with_gripper_rad,
                                                    const VectorXd &q_max_with_gripper_rad)
{
    if (current_status_ == STATUS::INITIALIZED)
        throw std::runtime_error(std::string(__func__) + ": the joint limits can not be changed while the driver is initialized.");
    if (q_min_with_gripper_rad.size() != kNumArmJoints+1 || q_max_with_gripper_rad.size() != kNumArmJoints+1)
        throw std::invalid_argument(std::string(__func__) + ": expected vectors of size 7, got sizes "
                                    + std::to_string(q_min_with_gripper_rad.size()) + " and "
                                    + std::to_string(q_max_with_gripper_rad.size()));
    for (Eigen::Index i=0;i<kNumArmJoints+1;i++)
    {
        if (q_min_with_gripper_rad(i) > q_max_with_gripper_rad(i))
            throw std::invalid_argument(std::string(__func__) + ": the lower limit of joint " + std::to_string(i)
                                        + " (" + std::to_string(q_min_with_gripper_rad(i)) + ") is greater than its upper limit ("
                                        + std::to_string(q_max_with_gripper_rad(i)) + ").");
    }
    q_min_ = q_min_with_gripper_rad.head(kNumArmJoints);
    q_max_ = q_max_with_gripper_rad.head(kNumArmJoints);
    gripper_min_ = q_min_with_gripper_rad(kNumArmJoints);
    gripper_max_ = q_max_with_gripper_rad(kNumArmJoints);
}

/**
 * @brief DriverUnitreeZ1::_compute_control_inputs computes the position commands. This method computes the joint velocity commands
 *                  and performs a numerical integration to command the robot in joint positions.
 *                  However, the solution is not sent to the robot.
 *
 * @param q       Current robot configuration.
 * @param qtarget Desired robot configuration.
 * @param gain    Convergency factor.
 * @return        The joint position control and the joint velocity solutions.
 */
std::tuple<VectorXd, VectorXd> DriverUnitreeZ1::_compute_control_inputs(VectorXd &q,
                                                                             const VectorXd &qtarget,
                                                                             const double &gain)
{
    double nq = gain;
    VectorXd f = 2*gain*(q-qtarget);
    MatrixXd Aeq;
    VectorXd beq;
    // Joint limit constraints A_*u <= b, with A_ = [-I; I; -I; I]:
    //   -u <= nq*(q - q_min)    (lower configuration limit)
    //    u <= nq*(q_max - q)    (upper configuration limit)
    //   -u <= q_dot_max         (lower configuration velocity limit)
    //    u <= q_dot_max         (upper configuration velocity limit)
    VectorXd b(4*kNumArmJoints);
    b << nq*(q-q_min_), nq*(q_max_-q), q_dot_max_, q_dot_max_;
    VectorXd u = solver_->solve_quadratic_program(H_, f, A_, b, Aeq, beq);
    q = q + T_*u;
    return {q, u};
}

/**
 * @brief DriverUnitreeZ1::_show_status displays the status of the driver if the verbosity flag is true.
 */
void DriverUnitreeZ1::_show_status()
{
    if (verbosity_)
        std::cerr<<status_msg_<<std::endl;
}

/**
 * @brief DriverUnitreeZ1::_should_shutdown checks whether a shutdown has been requested through the shutdown signaler.
 * @return true if a shutdown has been requested.
 */
bool DriverUnitreeZ1::_should_shutdown() const
{
    return shutdown_signaler_->should_shutdown();
}

/**
 * @brief DriverUnitreeZ1::_sample_robot_state updates the robot state during kStateSamplesBeforeIntegration samples,
 *                          to make sure the measured state is recent.
 */
void DriverUnitreeZ1::_sample_robot_state()
{
    marinholab::sas::core::Clock clock{T_, false};
    clock.init();
    for (int i=0;i<kStateSamplesBeforeIntegration;i++) // Updated the robot configuration
    {
        _update_robot_state();
        clock.update_and_sleep();
    }
}

/**
 * @brief DriverUnitreeZ1::_update_q_for_numerical_integration updates the robot state including the gripper position,
 *                          and initializes the numerical integration with the measured configuration.
 *                          This method must be called before to start the joint position control loop.
 */
void DriverUnitreeZ1::_update_q_for_numerical_integration()
{
    _sample_robot_state();
    q_ni_ = _get_measured_joint_positions();
    q_gripper_ni_ = _get_measured_gripper_position();
}


/**
 * @brief DriverUnitreeZ1::_set_driver_mode set the mode of the driver. This method throws an exception if a mode is not supported.
 * @param mode The driver mode. The following modes are supported:
 *
 *          PositionControl: Use this mode for command the robot using joint positions without specifiying joint velocities.
 *                          In this mode, the driver uses a QP to compute the velocity commands given the target joint positions.
 *                          The QP takes into account configuration and configuration velocity limits.
 *                          Use this method for command the robot using ROS 2 commands in the terminal.
 *
 *          RawPositionControl: Use this mode for command the robot using joint positions and joint velocities. There is no QP here
 *                              and the user must take into account configuration and configuration velocity limits. Use this method
 *                              if you use a kinematic controller to generate the control inputs.
 *
 */
void DriverUnitreeZ1::_set_driver_mode(const MODE &mode)
{
    switch (mode){

    case MODE::None:
        mode_ = mode;
        break;
    case MODE::PositionControl:
        mode_ = mode;
        break;
    case MODE::RawPositionControl:
        mode_ = mode;
        break;
    case MODE::VelocityControl:
        throw std::runtime_error("RobotDriverUnitreeZ1::_set_driver_mode: VelocityControl is unsupported");
        break;
    case MODE::ForceControl:
        throw std::runtime_error("RobotDriverUnitreeZ1::_set_driver_mode: ForceControl is unsupported");
        break;
    }
}

/**
 * @brief DriverUnitreeZ1::_update_robot_state updates the robot state.
 */
void DriverUnitreeZ1::_update_robot_state()
{
    const auto& lowstate = impl_->arm_->lowstate;
    std::scoped_lock lock(state_mutex_);
    q_measured_     = lowstate->getQ();
    q_dot_measured_ = lowstate->getQd();
    q_dot_dot_measured_ = lowstate->getQdd();
    tau_measured_  = lowstate->getTau();

    motor_temperatures_ = lowstate->temperature;

    gripper_position_measured_ = lowstate->getGripperQ();
    gripper_velocity_measured_ = lowstate->getGripperQd();
    gripper_torque_measured_   = lowstate->getGripperTau();
    /*
     * 0x01 : phase current is too large
     * 0x02 : phase leakage
     * 0x04 : motor winding overheat or temperature is too large
     * 0x20 : parameters jump
     * 0x40 : Ignore
     */
    errorstate_ = lowstate->errorstate;
    /*
     * 0: OK
     * 1: communication between lower computer and motor disconnect once
     * 2: communication between lower computer and motor has CRC erro once
     */
    motor_connection_state_ = lowstate->isMotorConnected;

    _check_motor_health();
}

/**
 * @brief DriverUnitreeZ1::_check_motor_health checks the motor error and communication states. Changes are logged
 *                  when they happen. A fault requests a shutdown, and is latched in motor_fault_detected_:
 *                  - The error flags 0x01 (phase current), 0x02 (phase leakage) and 0x04 (overheat) are faults.
 *                  - The other error flags are warnings, except 0x40, which the SDK documents as "Ignore".
 *                  - A communication error is a fault only if it lasts kMaxConsecutiveMotorConnectionErrors
 *                    consecutive state updates.
 *                  If the gripper is not attached, the gripper motor (last entry) is not checked.
 *
 * @pre The caller must hold state_mutex_.
 */
void DriverUnitreeZ1::_check_motor_health()
{
    if (last_reported_errorstate_.size() != errorstate_.size())
        last_reported_errorstate_.assign(errorstate_.size(), 0);
    if (last_reported_motor_connection_state_.size() != motor_connection_state_.size())
    {
        last_reported_motor_connection_state_.assign(motor_connection_state_.size(), 0);
        consecutive_motor_connection_faults_.assign(motor_connection_state_.size(), 0);
    }

    // Without the gripper, the last entry (the gripper motor) is not meaningful.
    const size_t ignored_entries = gripper_attached_ ? 0 : 1;
    const size_t number_of_error_entries = errorstate_.size() > ignored_entries ? errorstate_.size()-ignored_entries : 0;
    const size_t number_of_connection_entries = motor_connection_state_.size() > ignored_entries ?
                                                    motor_connection_state_.size()-ignored_entries : 0;
    std::string fault_reason;

    for (size_t i=0;i<number_of_error_entries;i++)
    {
        const uint8_t error = errorstate_.at(i) & static_cast<uint8_t>(~kMotorIgnoredErrorMask);
        if (error != last_reported_errorstate_.at(i))
        {
            if (error != 0)
                std::cerr<<"DriverUnitreeZ1: motor "<<i<<" reports the error state "<<to_hex(error)
                         <<(((error & kMotorFaultMask) != 0) ? " (fault)." : " (warning).")<<std::endl;
            else
                std::cerr<<"DriverUnitreeZ1: the error state of motor "<<i<<" was cleared."<<std::endl;
            last_reported_errorstate_.at(i) = error;
        }
        if ((error & kMotorFaultMask) != 0)
            fault_reason = "motor " + std::to_string(i) + " reports the error state " + to_hex(error);
    }

    for (size_t i=0;i<number_of_connection_entries;i++)
    {
        const uint8_t connection_state = motor_connection_state_.at(i);
        if (connection_state != last_reported_motor_connection_state_.at(i))
        {
            if (connection_state != 0)
                std::cerr<<"DriverUnitreeZ1: motor "<<i<<" reports the communication error "
                         <<static_cast<int>(connection_state)<<"."<<std::endl;
            else
                std::cerr<<"DriverUnitreeZ1: the communication with motor "<<i<<" is OK again."<<std::endl;
            last_reported_motor_connection_state_.at(i) = connection_state;
        }
        consecutive_motor_connection_faults_.at(i) = (connection_state != 0) ?
            std::min(consecutive_motor_connection_faults_.at(i)+1, kMaxConsecutiveMotorConnectionErrors) : 0;
        if (consecutive_motor_connection_faults_.at(i) >= kMaxConsecutiveMotorConnectionErrors)
            fault_reason = "motor " + std::to_string(i) + " reports communication errors in "
                           + std::to_string(kMaxConsecutiveMotorConnectionErrors) + " consecutive state updates";
    }

    if (!fault_reason.empty() && !motor_fault_detected_.exchange(true))
    {
        std::cerr<<"DriverUnitreeZ1: "<<fault_reason<<". Requesting shutdown."<<std::endl;
        shutdown_signaler_->shutdown();
    }
}

/**
 * @brief DriverUnitreeZ1::_get_measured_joint_positions returns the measured joint positions.
 * @return a vector 6x1 containing the joint positions, without the gripper position.
 */
VectorXd DriverUnitreeZ1::_get_measured_joint_positions()
{
    std::scoped_lock lock(state_mutex_);
    return q_measured_;
}

/**
 * @brief DriverUnitreeZ1::_get_measured_gripper_position returns the measured gripper position.
 * @return the gripper position in radians.
 */
double DriverUnitreeZ1::_get_measured_gripper_position()
{
    std::scoped_lock lock(state_mutex_);
    return gripper_position_measured_;
}


/**
 * @brief DriverUnitreeZ1::_start_state_thread starts the thread that updates the robot state.
 *                      This thread can not command the robot. It is only for receive information
 *                      from the robot.
 */
void DriverUnitreeZ1::_start_state_thread()
{
    if (state_thread_ && state_thread_->is_running())
        return;

    state_thread_ = std::make_unique<marinholab::sas::core::ThreadManager>(
        "z1_state",
        T_,
        [this](){_update_robot_state();},
        marinholab::sas::core::ThreadManager::PRIORITY::NORMAL,
        -1);
    state_thread_->start();
}

/**
 * @brief DriverUnitreeZ1::_stop_state_thread stops and releases the state thread, if one is running.
 */
void DriverUnitreeZ1::_stop_state_thread()
{
    if (state_thread_)
    {
        state_thread_->stop();
        state_thread_.reset();
    }
}


/**
 * @brief DriverUnitreeZ1::_send_arm_command sends a joint position and joint velocity command to the robot.
 *                  Every arm command goes through this method. The joint positions are clamped to [q_min_, q_max_]
 *                  and the joint velocities to [-q_dot_max_, q_dot_max_]. The velocity of a joint whose position
 *                  was clamped is set to zero, so the robot does not push against the limit.
 * @param q The target joint positions.
 * @param q_dot The target joint velocities.
 */
void DriverUnitreeZ1::_send_arm_command(const VectorXd &q, const VectorXd &q_dot)
{
    VectorXd q_clamped = q.cwiseMax(q_min_).cwiseMin(q_max_);
    VectorXd q_dot_clamped = q_dot.cwiseMax(-q_dot_max_).cwiseMin(q_dot_max_);
    for (Eigen::Index i=0;i<kNumArmJoints;i++)
    {
        if (q_clamped(i) != q(i))
            q_dot_clamped(i) = 0.0;
    }
    impl_->arm_->setArmCmd(q_clamped, q_dot_clamped);
    last_commanded_joint_positions_ = q_clamped;
}

/**
 * @brief DriverUnitreeZ1::_send_gripper_command sends a gripper position and gripper velocity command to the robot.
 *                  The gripper position is clamped to [gripper_min_, gripper_max_]. If the position was clamped,
 *                  the gripper velocity is set to zero.
 * @param q_gripper The target gripper position.
 * @param q_gripper_dot The target gripper velocity.
 */
void DriverUnitreeZ1::_send_gripper_command(const double &q_gripper, const double &q_gripper_dot)
{
    const double q_gripper_clamped = std::clamp(q_gripper, gripper_min_, gripper_max_);
    const double q_gripper_dot_clamped = (q_gripper_clamped != q_gripper) ? 0.0 : q_gripper_dot;
    impl_->arm_->setGripperCmd(q_gripper_clamped, q_gripper_dot_clamped);
}

/**
 * @brief DriverUnitreeZ1::_gripper_control_step moves the gripper towards the target gripper position, which is
 *                  clamped to [gripper_min_, gripper_max_].
 */
void DriverUnitreeZ1::_gripper_control_step()
{
    const double target_gripper_position = std::clamp(get_target_gripper_position(), gripper_min_, gripper_max_);
    const double q_gripper_position_dot = -gain_gripper_*(q_gripper_ni_-target_gripper_position);
    q_gripper_ni_ = q_gripper_ni_ + T_*q_gripper_position_dot;
    _send_gripper_command(q_gripper_ni_, q_gripper_position_dot);
}

/**
 * @brief DriverUnitreeZ1::_hold_last_command commands the robot to stay at the last commanded configuration,
 *                      with zero velocity. Call it only after the control thread has commanded the robot,
 *                      otherwise the last commanded configuration is not meaningful.
 */
void DriverUnitreeZ1::_hold_last_command()
{
    _send_arm_command(last_commanded_joint_positions_, VectorXd::Zero(kNumArmJoints));
    if (gripper_attached_)
        _send_gripper_command(q_gripper_ni_, 0.0);
}


/**
 * @brief DriverUnitreeZ1::_joint_position_control_step updates the robot command in the PositionControl mode.
 */
void DriverUnitreeZ1::_joint_position_control_step()
{
    auto results = _compute_control_inputs(q_ni_, get_target_joint_positions(), kPositionControlGain);
    q_ni_ = std::get<0>(results);
    _send_arm_command(q_ni_, std::get<1>(results));

    if (gripper_attached_)
        _gripper_control_step();
}


/**
 * @brief DriverUnitreeZ1::_joint_raw_position_control_step updates the robot command in the RawPositionControl mode.
 *
 *                      This mode is meant to be used with a kinematic control package that ensures smooth
 *                      commands. The targets are sent as is, without a QP or rate limiting. Only the clamping
 *                      to the configuration and configuration velocity limits is applied, as a safety net.
 */
void DriverUnitreeZ1::_joint_raw_position_control_step()
{
    _send_arm_command(get_target_joint_positions(), get_target_joint_velocities());

    if (gripper_attached_)
        _gripper_control_step();
}


/**
 * @brief DriverUnitreeZ1::_control_loop_callback runs one tick of the control thread: updates the robot state and
 *                      commands the robot according to the operation mode. Once a shutdown has been requested,
 *                      every tick holds the last commanded configuration until deinitialize() stops the thread.
 *                      The thread is not stopped from here, because stopping a ThreadManager from its own
 *                      callback would deadlock.
 */
void DriverUnitreeZ1::_control_loop_callback()
{
    try
    {
        _update_robot_state();

        if (_should_shutdown())
        {
            _hold_last_command();
            return;
        }

        switch (mode_) {
        case MODE::PositionControl:
            _joint_position_control_step();
            break;
        case MODE::RawPositionControl:
            _joint_raw_position_control_step();
            break;
        case MODE::None:
        case MODE::VelocityControl:
        case MODE::ForceControl:
            break;
        }
    }
    catch (const std::exception& e)
    {
        // Nothing can catch an exception thrown from this thread, so stop the robot driver instead.
        std::cerr<<"DriverUnitreeZ1::_control_loop_callback: "<<e.what()<<". Requesting shutdown."<<std::endl;
        shutdown_signaler_->shutdown();
    }
}


/**
 * @brief DriverUnitreeZ1::_start_control_thread initializes the numerical integration and the targets with the
 *                      measured robot configuration, and starts the control thread.
 */
void DriverUnitreeZ1::_start_control_thread()
{
    if (control_thread_ && control_thread_->is_running())
        return;

    _update_q_for_numerical_integration();
    set_target_joint_positions(q_ni_);
    set_target_joint_velocities(VectorXd::Zero(kNumArmJoints));
    set_target_gripper_position(q_gripper_ni_);
    last_commanded_joint_positions_ = q_ni_;

    control_thread_ = std::make_unique<marinholab::sas::core::ThreadManager>(
        "z1_control",
        T_,
        [this](){_control_loop_callback();},
        marinholab::sas::core::ThreadManager::PRIORITY::NORMAL,
        -1);
    control_thread_->start();
    status_msg_ = (mode_ == MODE::PositionControl) ? "Starting joint position control." :
                                                      "Starting joint raw position control.";
    _show_status();
}

/**
 * @brief DriverUnitreeZ1::_stop_control_thread stops and releases the control thread, if one is running.
 *                      When this method returns, the control thread no longer commands the robot.
 */
void DriverUnitreeZ1::_stop_control_thread()
{
    if (control_thread_)
    {
        control_thread_->stop();
        control_thread_.reset();
        status_msg_ = "Control thread finished.";
        _show_status();
    }
}


/**
 * @brief DriverUnitreeZ1::connect This method establishes the connection with robot. This is the first method to be called
 *                      after the constructor of the class.
 */
void DriverUnitreeZ1::connect()
{
    if (current_status_ == STATUS::IDLE)
    {
        impl_->arm_->sendRecvThread->start();
        T_ = impl_->arm_->_ctrlComp->dt;

        _start_state_thread();
        current_status_ = STATUS::CONNECTED;
        status_msg_ = "connected!";
        _show_status();
    }
}


/**
 * @brief DriverUnitreeZ1::initialize This method starts the appropriate threads based on the selected mode of operation.
 *                  This method requires established communication with the robot, i.e. the user must call connect()
 *                  before calling initialize().
 *                  If the operation mode is different from None, the robot may move!
 *                  WARNING: Be prepared to stop the robot with an emergency stop protocol!
 */
void DriverUnitreeZ1::initialize()
{
    if (current_status_ == STATUS::CONNECTED)
    {
        // Check the motors before moving the robot.
        _sample_robot_state();
        if (motor_fault_detected_)
            throw std::runtime_error(std::string(__func__) + ": a motor fault was detected. The robot was not moved.");

        impl_->arm_->backToStart();
        impl_->arm_->startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);

        std::cout<<"Reading robot configuration..."<<std::endl;
        _update_q_for_numerical_integration();


        initial_robot_configuration_ = q_ni_;

        if (move_robot_to_initial_custom_configuration_when_initialized_)
        {
            std::cerr<<"Setting initial custom configuration..."<<std::endl;
            _move_robot_to_target_joint_positions(initial_configuration_, open_loop_joint_control_gain_, 0.01,
                                                  true, kBlockingMoveTimeoutSec);
        }

        switch (mode_) {
        case MODE::None:
            // No control thread. The state thread keeps updating the robot state.
            break;
        case MODE::PositionControl:
        case MODE::RawPositionControl:
            // The control thread updates the robot state from now on.
            _stop_state_thread();
            _start_control_thread();
            break;
        case MODE::VelocityControl:
            break;
        case MODE::ForceControl:
            break;

        }
        current_status_ = STATUS::INITIALIZED;
        status_msg_ = "initialized!";
        _show_status();
    }
}


/**
 * @brief DriverUnitreeZ1::deinitialize This method stops the control thread, moves the robot to the configuration it had
 *                  when initialize() was called, and then to its home configuration. After a motor fault, the move to
 *                  the configuration it had when initialize() was called is skipped.
 *                  This method requires an initialized driver, i.e. the user must call both connect(),
 *                  and initialize() before calling deinitialize(). Otherwise, this method does nothing.
 */
void DriverUnitreeZ1::deinitialize()
{
    if (current_status_ != STATUS::INITIALIZED)
        return;

    if (control_thread_)
    {
        std::cout<<"Ending control loop..."<<std::endl;
        _stop_control_thread();
        // Remove any velocity command left by the control thread before homing the robot.
        _hold_last_command();
    }

    if (motor_fault_detected_)
    {
        // Do not command the robot with the driver after a motor fault. backToStart() is still used.
        std::cerr<<"A motor fault was detected. Skipping the move to the initial robot configuration."<<std::endl;
    }
    else
    {
        // This move ignores the shutdown signal on purpose, to move the robot to its initial configuration.
        std::cout<<"Setting home robot configuration..."<<std::endl;
        _move_robot_to_target_joint_positions(initial_robot_configuration_, open_loop_joint_control_gain_, 0.015,
                                              false, kBlockingMoveTimeoutSec);
    }

    impl_->arm_->backToStart();
    impl_->arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    status_msg_ = "Deinitialized.";
    current_status_ = STATUS::DEINITIALIZED;
    _show_status();
}


/**
 * @brief DriverUnitreeZ1::disconnect performs the last steps to disconnect the driver. If the driver is still
 *              initialized, this method calls deinitialize() first.
 */
void DriverUnitreeZ1::disconnect()
{
    if (current_status_ == STATUS::INITIALIZED)
        deinitialize();

    if (current_status_ != STATUS::CONNECTED && current_status_ != STATUS::DEINITIALIZED)
        return;

    status_msg_ = "Disconnecting...";
    _show_status();
    _stop_control_thread();
    _stop_state_thread();
    impl_->arm_->sendRecvThread->shutdown();

    status_msg_ = "Disconnected.";
    current_status_ = STATUS::DISCONNECTED;
    _show_status();
}



/**
 * @brief DriverUnitreeZ1::_move_robot_to_target_joint_positions commands the robot in joint position level, in the
 *                      caller thread. This method must not be called while the control thread is running.
 *
 * @param q_target   The desired robot configuration.
 * @param gain       Convergence factor
 * @param error_norm The error norm used to break the control loop.
 * @param stop_on_shutdown Use true to break the control loop when a shutdown is requested.
 * @param timeout_sec The maximum duration of the motion, in seconds.
 * @return true if the robot reached the desired configuration. false if the motion was stopped by a shutdown
 *         request or by the timeout.
 */
bool DriverUnitreeZ1::_move_robot_to_target_joint_positions(const VectorXd &q_target,
                                                            const double &gain,
                                                            const double &error_norm,
                                                            const bool &stop_on_shutdown,
                                                            const double &timeout_sec)
{
    _update_q_for_numerical_integration();
    marinholab::sas::core::Clock clock{T_, false};
    clock.init();
    while ((q_ni_-q_target).norm() > error_norm)
    {
        if (stop_on_shutdown && _should_shutdown())
            return false;
        if (clock.get_elapsed_time_sec() > timeout_sec)
        {
            std::cerr<<std::string(__func__)<<": the target configuration was not reached within "
                     <<timeout_sec<<" seconds. Remaining error norm: "<<(q_ni_-q_target).norm()<<std::endl;
            return false;
        }
        _update_robot_state();
        auto results = _compute_control_inputs(q_ni_, q_target, gain);
        q_ni_ = std::get<0>(results);

        // To move the robot in joint position commands, we need to define both a target robot configuration and a target
        // robot configuration velocity.
        _send_arm_command(q_ni_, std::get<1>(results));
        clock.update_and_sleep();
    }
    return true;
}

/**
 * @brief DriverUnitreeZ1::get_joint_positions returns the meausured joint positions.
 * @return a vector containing the joint positions. This vector is 6x1 and does not include the gripper position.
 */
VectorXd DriverUnitreeZ1::get_joint_positions()
{
    return _get_measured_joint_positions();
}

/**
 * @brief DriverUnitreeZ1::get_joint_velocities returns the measured joint velocities.
 * @return a vector containing the joint velocities. This vector is 6x1 and does not include the gripper velocity.
 */
VectorXd DriverUnitreeZ1::get_joint_velocities()
{
    std::scoped_lock lock(state_mutex_);
    return q_dot_measured_;
}

/**
 * @brief DriverUnitreeZ1::get_joint_velocities_with_gripper returns the measured joint velocities.
 * @return a vector containing the joint velocities. This vector is 7x1 and includes the gripper velocity.
 */
VectorXd DriverUnitreeZ1::get_joint_velocities_with_gripper()
{
    std::scoped_lock lock(state_mutex_);
    VectorXd joint_velocities_with_gripper = VectorXd::Zero(7);
    joint_velocities_with_gripper << q_dot_measured_(0), q_dot_measured_(1), q_dot_measured_(2),
                                     q_dot_measured_(3), q_dot_measured_(4), q_dot_measured_(5), gripper_velocity_measured_;
    return joint_velocities_with_gripper;
}


/**
 * @brief DriverUnitreeZ1::get_joint_torques_with_gripper returns the measured joint torques.
 * @return a vector containing the joint torques. This vector is 7x1 and includes the gripper torque.
 */
VectorXd DriverUnitreeZ1::get_joint_torques_with_gripper()
{
    std::scoped_lock lock(state_mutex_);
    VectorXd joint_torques_with_gripper = VectorXd::Zero(7);
    joint_torques_with_gripper << tau_measured_(0), tau_measured_(1), tau_measured_(2),
        tau_measured_(3), tau_measured_(4), tau_measured_(5), gripper_torque_measured_;
    return joint_torques_with_gripper;
}

/**
 * @brief DriverUnitreeZ1::get_joint_forces returns the estimated joint forces.
 * @return a vector containing the joint forces. This vector is 6x1 and does not include the gripper effort.
 */
VectorXd DriverUnitreeZ1::get_joint_forces()
{
    std::scoped_lock lock(state_mutex_);
    return tau_measured_;
}

/**
 * @brief DriverUnitreeZ1::get_gripper_position returns the gripper position in radians.
 * @return The gripper position. If the robot is started in the home configuration, A value of zero denotes the gripper closed.
 *         A negative value denotes the gripper opened.
 */
double DriverUnitreeZ1::get_gripper_position()
{
    return _get_measured_gripper_position();
}


/**
 * @brief DriverUnitreeZ1::move_to_initial_configuration_when_initialized set a flag used to move the robot a custom configuration,
 *                      before starting the control loop.
 * @param flag Use true if you want to move the robot to a custom configuration before initializing the driver.
 */
void DriverUnitreeZ1::move_to_initial_configuration_when_initialized(const bool &flag)
{
    move_to_initial_configuration_when_initialized(flag,
                                                   FORWARD_CONFIGURATION_);
}


/**
 * @brief DriverUnitreeZ1::move_to_initial_configuration_when_initialized set a flag used to move the robot a custom configuration,
 *                      before starting the control loop.
 *                      This method must be called after connect() and before initialize().
 * @param flag Use true if you want to move the robot to a custom configuration before initializing the driver.
 * @param initial_configuration The custom initial configuration.
 */
void DriverUnitreeZ1::move_to_initial_configuration_when_initialized(const bool &flag,
                                                                     const VectorXd& initial_configuration)
{
    move_robot_to_initial_custom_configuration_when_initialized_ = flag;
    if (initial_configuration.size() != FORWARD_CONFIGURATION_ .size())
        throw std::runtime_error(std::string(__func__)+ ": Wrong initial configuration size!");
    initial_configuration_ = initial_configuration;
}


/**
 * @brief DriverUnitreeZ1::move_to_target_joint_positions moves the robot to a desired robot configuration.
 *                      This method is designed to set a start robot configuration before to start a control loop.
 *                      If the operation mode is None, the method works, but this feature is going to be removed.
 *                      If the operation mode is PositionControl, the driver must be connected but not initialized.
 * @param q_target The desired robot configuration.
 */
void DriverUnitreeZ1::move_to_target_joint_positions(const VectorXd &q_target)
{
    if (q_target.size() != kNumArmJoints)
        throw std::invalid_argument(std::string(__func__) + ": expected a vector of size 6, got size "
                                    + std::to_string(q_target.size()));

    if (current_status_ == STATUS::INITIALIZED && mode_ == MODE::None)
    {
        _move_robot_to_target_joint_positions(q_target, 0.1, 0.01, true, kBlockingMoveTimeoutSec);
    }else if( mode_ == MODE::PositionControl && current_status_ == STATUS::CONNECTED)
    {
        _move_robot_to_target_joint_positions(q_target, 0.1, 0.01, true, kBlockingMoveTimeoutSec);
    }
    else
    {
        std::cerr<<"This method is enabled for MODE::None and STATUS::INITIALIZED"<<std::endl;
        std::cerr<<"Or MODE::PositionControl and STATUS::CONNECTED."<<std::endl;
    }
}

/**
 * @brief DriverUnitreeZ1::set_target_joint_positions sets the target joint positions.
 * @param target_joint_positions_rad The target joint positions in radians. This vector must be 6x1.
 */
void DriverUnitreeZ1::set_target_joint_positions(const VectorXd &target_joint_positions_rad)
{
    if (target_joint_positions_rad.size() != kNumArmJoints)
        throw std::invalid_argument(std::string(__func__) + ": expected a vector of size 6, got size "
                                    + std::to_string(target_joint_positions_rad.size()));
    std::scoped_lock lock(target_mutex_); // Locks acquired
    target_joint_positions_ = target_joint_positions_rad;
}


/**
 * @brief DriverUnitreeZ1::get_target_joint_positions gets the target joint positions.
 * @return The target joint positions
 */
VectorXd DriverUnitreeZ1::get_target_joint_positions()
{
    std::scoped_lock lock(target_mutex_); // Locks acquired
    return target_joint_positions_;
}

/**
 * @brief DriverUnitreeZ1::set_target_joint_velocities sets the target joint velocities.
 * @param target_joint_velocities_rad_s The target joint velocities in rad/s. This vector must be 6x1.
 */
void DriverUnitreeZ1::set_target_joint_velocities(const VectorXd &target_joint_velocities_rad_s)
{
    if (target_joint_velocities_rad_s.size() != kNumArmJoints)
        throw std::invalid_argument(std::string(__func__) + ": expected a vector of size 6, got size "
                                    + std::to_string(target_joint_velocities_rad_s.size()));
    std::scoped_lock lock(target_mutex_);
    target_joint_velocities_ = target_joint_velocities_rad_s;
}

/**
 * @brief DriverUnitreeZ1::get_target_joint_velocities gets the target joint  velocities
 * @return The target joint velocities
 */
VectorXd DriverUnitreeZ1::get_target_joint_velocities()
{
    std::scoped_lock lock(target_mutex_);
    return target_joint_velocities_;
}



/**
 * @brief DriverUnitreeZ1::set_target_gripper_position sets the gripper position when the operation mode is PositionControl.
 * @param target_gripper_position The target gripper position in radians. If the robot is correctly started following the official instructions,
 *                      a value of zero represents a closed gripper. A value lower than zero is to open the gripper.
 */
void DriverUnitreeZ1::set_target_gripper_position(const double &target_gripper_position)
{
    std::scoped_lock lock(target_mutex_);
    target_gripper_position_ = target_gripper_position;
}

/**
 * @brief DriverUnitreeZ1::get_target_gripper_position gets the target gripper position
 * @return The target gripper position.
 */
double DriverUnitreeZ1::get_target_gripper_position()
{
    std::scoped_lock lock(target_mutex_);
    return target_gripper_position_;
}

/**
 * @brief DriverUnitreeZ1::set_target_joint_positions_with_gripper sets the target joint positions and the target gripper position
 *                  when the operation mode is PositionControl.
 * @param target_joint_positions_with_gripper_rad A vector of 7x1 containing the desired joint positions including the gripper.
 */
void DriverUnitreeZ1::set_target_joint_positions_with_gripper(const VectorXd &target_joint_positions_with_gripper_rad)
{
    if (target_joint_positions_with_gripper_rad.size() != kNumArmJoints+1)
        throw std::invalid_argument(std::string(__func__) + ": expected a vector of size 7, got size "
                                    + std::to_string(target_joint_positions_with_gripper_rad.size()));
    set_target_joint_positions(target_joint_positions_with_gripper_rad.head(kNumArmJoints));
    set_target_gripper_position(target_joint_positions_with_gripper_rad(kNumArmJoints));
}


/**
 * @brief DriverUnitreeZ1::set_target_joint_velocities_with_gripper sets the target joint velocities and the target gripper velocity
 * @param target_joint_velocities_with_gripper_rad  A vector of 7x1 containing the desired joint velocities including the gripper.
 */
void DriverUnitreeZ1::set_target_joint_velocities_with_gripper(const VectorXd &target_joint_velocities_with_gripper_rad)
{
    if (target_joint_velocities_with_gripper_rad.size() != kNumArmJoints+1)
        throw std::invalid_argument(std::string(__func__) + ": expected a vector of size 7, got size "
                                    + std::to_string(target_joint_velocities_with_gripper_rad.size()));
    set_target_joint_velocities(target_joint_velocities_with_gripper_rad.head(kNumArmJoints));
}

/**
 * @brief DriverUnitreeZ1::get_joint_positions_with_gripper returns the measured joint positions including the gripper.
 * @return A vector 7x1 containing the joint positions and the gripper position.
 */
VectorXd DriverUnitreeZ1::get_joint_positions_with_gripper()
{
    std::scoped_lock lock(state_mutex_);
    VectorXd joint_positions_with_gripper = VectorXd::Zero(7);
    joint_positions_with_gripper << q_measured_(0), q_measured_(1), q_measured_(2),
                                    q_measured_(3), q_measured_(4), q_measured_(5), gripper_position_measured_;
    return joint_positions_with_gripper;
}




