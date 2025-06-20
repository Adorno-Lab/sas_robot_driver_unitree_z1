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
#include <thread>
#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <memory>

using namespace DQ_robotics;
using namespace Eigen;



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
        PositionControl,
        VelocityControl,
        ForceControl,
    };
protected:
    std::atomic_bool* st_break_loops_;
private:
    class Impl;
    std::shared_ptr<Impl> impl_;

    STATUS current_status_{STATUS::IDLE};
    MODE mode_{MODE::None};
    std::string status_msg_;
    double T_;

    std::unique_ptr<DQ_QPOASESSolver> solver_;
    MatrixXd H_;
    MatrixXd I_;

    std::tuple<VectorXd, VectorXd> _compute_control_inputs(VectorXd& q,
                                                           const VectorXd& qtarget,
                                                           const double& gain);


    VectorXd q_min_ = (VectorXd(6) << -2.6180, 0, -2.7925, -1.3963, -1.4835, -2.7925).finished();
    VectorXd q_max_ = (VectorXd(6) <<  2.6180, 3.1416, 0, 1.3963, 1.4835, 2.7925).finished();
    VectorXd q_dot_max_ = (VectorXd(6) << 3.1416, 3.1416, 3.1416, 3.1416, 3.1416, 3.1416).finished();

    VectorXd q_ni_ = VectorXd::Zero(6); //For numerical integrations
    VectorXd q_measured_ = VectorXd::Zero(6);
    VectorXd q_dot_measured_ = VectorXd::Zero(6);
    VectorXd q_dot_dot_measured_ = VectorXd::Zero(6);
    VectorXd tau_measured_ = VectorXd::Zero(6);
    VectorXi motor_temperatures_;
    std::vector<uint8_t>  errorstate_;

    VectorXd Forward_ = (VectorXd(6) << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0).finished();
    bool move_robot_to_forward_position_when_initialized_{false};

    void _show_status();
    bool verbosity_;

    void _update_q_for_numerical_integration();

    VectorXd target_joint_positions_;
    VectorXd initial_robot_configuration_;
    void _set_driver_mode(const MODE& mode);

    void _update_robot_state();

    double q_gripper_ni_{0}; // For numerical integration;
    double gripper_position_measured_;
    double gripper_velocity_measured_;
    double target_gripper_position_{0};
    bool gripper_attached_;

    //-------To handle the threads-----------------
    void _echo_robot_state_mode();
    std::thread echo_robot_state_mode_thread_;
    void _start_echo_robot_state_mode_thread();
    std::atomic<bool> finish_echo_robot_state_;
    void _finish_echo_robot_state();


    void _joint_position_control_mode();
    std::thread joint_position_control_mode_thread_;
    void _start_joint_position_control_thread();
    std::atomic<bool> finish_motion_;
    void _finish_motion();

    void _move_robot_to_target_joint_positions(const VectorXd& q_target, const double& gain, std::atomic_bool* break_loop);


public:
    DriverUnitreeZ1() = delete;
    DriverUnitreeZ1(const DriverUnitreeZ1&) = delete;
    DriverUnitreeZ1& operator= (const DriverUnitreeZ1&) = delete;
    DriverUnitreeZ1(std::atomic_bool* st_break_loops,
                         const MODE& mode = MODE::None,
                         const bool& gripper_attached = true,
                         const bool& verbosity = true);

   // DriverUnitreeZ1(const RobotDriverUnitreeZ1Configuration& configuration, std::atomic_bool* break_loops);


    void connect();
    void initialize();
    void deinitialize();
    void disconnect();


    void set_target_gripper_position(const double& target_gripper_position);


    VectorXd get_joint_positions();
    VectorXd get_joint_velocities();
    VectorXd get_joint_velocities_with_gripper();
    VectorXd get_joint_forces();
    double get_gripper_position();

    void move_robot_to_forward_position_when_initialized(const bool& flag = true);

    void move_robot_to_target_joint_positions(const VectorXd& q_target);

    void set_target_joint_positions(const VectorXd& target_joint_positions_rad);

    void set_gripper_position(const double& gripper_position);
    void set_target_joint_positions_with_gripper(const VectorXd& target_joint_positions_with_gripper_rad);
    VectorXd get_joint_positions_with_gripper();

};


