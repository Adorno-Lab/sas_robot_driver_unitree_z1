/**
(C) Copyright 2024-2025 Adorno-Lab software developments

    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License.
    If not, see <http://www.gnu.org/licenses/>.

Contributors:

1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
        - Responsible for the original implementation.
*/

#pragma once
#include <vector>
#include <atomic>
#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <marinholab/sas/core/sas_shutdown_signaler.hpp>
#include <marinholab/sas/core/sas_thread_manager.hpp>
#include <memory>
#include <mutex>

using namespace DQ_robotics;
using namespace Eigen;



/**
 * @brief Driver for the Unitree Z1 arm (and the optional Unitree gripper), built on the Unitree Z1 SDK.
 *
 * @details Threading model:
 *          - The SDK runs its own UDP send/receive thread (unitreeArm::sendRecvThread), started in connect().
 *          - state_thread_ (a ThreadManager) only reads the robot state. It runs from connect() until a
 *            control mode takes over in initialize() (or for the whole session in MODE::None).
 *          - control_thread_ (a ThreadManager) reads the robot state and commands the robot according to
 *            the selected MODE. It runs from initialize() until deinitialize().
 *          - The blocking moves (initial configuration in initialize(), home configuration in deinitialize())
 *            run in the caller thread, and never while control_thread_ is running.
 *          The measured state is guarded by state_mutex_ and the targets by target_mutex_, so the getters and
 *          setters are safe to call from any thread.
 */
class DriverUnitreeZ1
{
public:
    enum class STATUS{
        IDLE,
        CONNECTED,
        INITIALIZED,
        DEINITIALIZED,
        DISCONNECTED,
    };

    enum class MODE{
        None,
        PositionControl,    // To command the robot using ros2 topic commands on the terminal
        RawPositionControl, // To command the robot using a kinematic control Strategy
        VelocityControl,
        ForceControl,
    };
private:
    class Impl;
    std::shared_ptr<Impl> impl_;

    std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler_;

    STATUS current_status_{STATUS::IDLE};
    MODE mode_{MODE::None};
    std::string status_msg_;
    double T_{0.002};

    std::unique_ptr<DQ_QPOASESSolver> solver_;
    MatrixXd H_;
    MatrixXd I_;

    double open_loop_joint_control_gain_{0.3};

    std::tuple<VectorXd, VectorXd> _compute_control_inputs(VectorXd& q,
                                                           const VectorXd& qtarget,
                                                           const double& gain);


    VectorXd q_min_ = (VectorXd(6) << -2.6180, 0, -2.7925, -1.3963, -1.4835, -2.7925).finished();
    VectorXd q_max_ = (VectorXd(6) <<  2.6180, 3.1416, 0, 1.3963, 1.4835, 2.7925).finished();
    VectorXd q_dot_max_ = (VectorXd(6) << 3.1416, 3.1416, 3.1416, 3.1416, 3.1416, 3.1416).finished();
    // Gripper limits in radians. Zero is the closed gripper and negative values open it. -0.6981 rad = -40 deg.
    double gripper_min_{-0.6981};
    double gripper_max_{0.0};

    //--------------Measured robot state (guarded by state_mutex_)--------------------
    std::mutex state_mutex_;
    VectorXd q_measured_ = VectorXd::Zero(6);
    VectorXd q_dot_measured_ = VectorXd::Zero(6);
    VectorXd q_dot_dot_measured_ = VectorXd::Zero(6);
    VectorXd tau_measured_ = VectorXd::Zero(6);
    // Motor status. These vectors have one entry per motor (8 entries for 7 joints, because one joint
    // is driven by two motors), so the motor index is not the joint index. The last entry is the gripper.
    std::vector<int> motor_temperatures_;
    std::vector<uint8_t>  errorstate_;
    std::vector<uint8_t>  motor_connection_state_;
    // Motor health tracking, only used by _check_motor_health().
    std::vector<uint8_t> last_reported_errorstate_;
    std::vector<uint8_t> last_reported_motor_connection_state_;
    std::vector<int> consecutive_motor_connection_faults_;
    double gripper_position_measured_{0};
    double gripper_velocity_measured_{0};
    double gripper_torque_measured_{0};
    //---------------------------------------------------------------------------------

    //--------------Targets (guarded by target_mutex_)---------------------------------
    // Based on https://github.com/MarinhoLab/sas_robot_driver_ur/blob/main/src/driver_ur_joint_positions_manager.hpp
    std::mutex target_mutex_;
    VectorXd target_joint_positions_;
    VectorXd target_joint_velocities_;
    double target_gripper_position_{0};
    //---------------------------------------------------------------------------------

    // Numerical integration state. Only used by one thread at a time: by the blocking moves
    // in the caller thread, or by control_thread_ while it is running.
    VectorXd q_ni_ = VectorXd::Zero(6);
    double q_gripper_ni_{0};

    const VectorXd FORWARD_CONFIGURATION_ = (VectorXd(6) << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0).finished();
    VectorXd initial_configuration_ = (VectorXd(6) << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0).finished();
    bool move_robot_to_initial_custom_configuration_when_initialized_{false};

    void _show_status();
    bool verbosity_;

    void _sample_robot_state();
    void _update_q_for_numerical_integration();

    VectorXd initial_robot_configuration_;
    void _set_driver_mode(const MODE& mode);

    void _update_robot_state();
    void _check_motor_health();
    std::atomic_bool motor_fault_detected_{false};
    VectorXd _get_measured_joint_positions();
    double _get_measured_gripper_position();

    bool gripper_attached_;
    double gain_gripper_{1.0};

    bool _should_shutdown() const;


    //-------To handle the threads-----------------
    std::unique_ptr<marinholab::sas::core::ThreadManager> state_thread_;
    void _start_state_thread();
    void _stop_state_thread();

    std::unique_ptr<marinholab::sas::core::ThreadManager> control_thread_;
    void _start_control_thread();
    void _stop_control_thread();
    void _control_loop_callback();
    void _joint_position_control_step();
    void _joint_raw_position_control_step();
    void _gripper_control_step();
    void _hold_last_command();
    VectorXd last_commanded_joint_positions_ = VectorXd::Zero(6);

    void _send_arm_command(const VectorXd& q, const VectorXd& q_dot);
    void _send_gripper_command(const double& q_gripper, const double& q_gripper_dot);

    bool _move_robot_to_target_joint_positions(const VectorXd& q_target,
                                               const double& gain,
                                               const double& error_norm,
                                               const bool& stop_on_shutdown,
                                               const double& timeout_sec);


public:
    DriverUnitreeZ1() = delete;
    DriverUnitreeZ1(const DriverUnitreeZ1&) = delete;
    DriverUnitreeZ1& operator= (const DriverUnitreeZ1&) = delete;
    DriverUnitreeZ1(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
                    const MODE& mode = MODE::None,
                    const bool& gripper_attached = true,
                    const bool& verbosity = true,
                    const double&  open_loop_joint_control_gain = 0.3);

    [[deprecated("Use DriverUnitreeZ1(const std::shared_ptr<ShutdownSignaler>&, ...) instead.")]]
    DriverUnitreeZ1(std::atomic_bool* st_break_loops,
                    const MODE& mode = MODE::None,
                    const bool& gripper_attached = true,
                    const bool& verbosity = true,
                    const double&  open_loop_joint_control_gain = 0.3);

    ~DriverUnitreeZ1();

    void set_joint_limits_with_gripper(const VectorXd& q_min_with_gripper_rad,
                                       const VectorXd& q_max_with_gripper_rad);

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();




    VectorXd get_joint_positions();
    VectorXd get_joint_velocities();
    VectorXd get_joint_forces();

    VectorXd get_joint_positions_with_gripper();
    VectorXd get_joint_velocities_with_gripper();
    VectorXd get_joint_torques_with_gripper();




    void move_to_initial_configuration_when_initialized(const bool& flag);
    void move_to_initial_configuration_when_initialized(const bool& flag,
                                                        const VectorXd& initial_configuration);

    void move_to_target_joint_positions(const VectorXd& q_target);

    void set_target_joint_positions(const VectorXd& target_joint_positions_rad);
    VectorXd get_target_joint_positions();

    void set_target_joint_velocities(const VectorXd& target_joint_velocities_rad_s);
    VectorXd get_target_joint_velocities();

    void set_target_gripper_position(const double& target_gripper_position);
    double get_target_gripper_position();

    double get_gripper_position();

    void set_target_joint_positions_with_gripper(const VectorXd& target_joint_positions_with_gripper_rad);
    void set_target_joint_velocities_with_gripper(const VectorXd& target_joint_velocities_with_gripper_rad);


};


