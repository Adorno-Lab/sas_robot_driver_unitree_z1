/*
# Copyright (c) 2025 Adorno-Lab
#
#    This file is part of sas_robot_driver_unitree_z1.
#
#    sas_robot_driver_unitree_z1 is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_unitree_z1 is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_unitree_z1.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#   Based on sas_robot_driver_ur.hpp 
#   (https://github.com/MarinhoLab/sas_robot_driver_ur/blob/main/include/sas_robot_driver_ur/sas_robot_driver_ur.hpp)
#
# ################################################################*/

#pragma once
#include <atomic>
#include <thread>

#include <sas_core/sas_robot_driver.hpp>

using namespace Eigen;
namespace sas
{

struct RobotDriverUnitreeZ1Configuration
{
    bool gripper_attached;  //const bool gripper_attached = = true;
    std::string mode;       //const std::string mode= "RawPositionControl";
    bool verbosity;         //const bool verbosity = true;
    std::tuple<VectorXd,VectorXd> joint_limits;
    bool move_to_initial_configuration;  // Use true if you want to move the robot to the initial configuration
    VectorXd initial_configuration;  // Custom initial configuration before starting the control loop
    double open_loop_joint_control_gain; //Convergence rate when moving the robot to the initial and home configuration
};

/*
Operation modes:
    RawPositionControl: ----> Use this mode when you command the robot using both joint position commands
                    and joint velocity commands.
                    You must take into account the configuration and configuration velocity limits.
        Example:

        // Initialize the RobotDriverClient
        sas::RobotDriverClient rdi(node, "/sas_z1/z1_1");

        clock.update_and_sleep();

        rdi.send_target_joint_positions(target_joint_positions);
        rdi.send_target_joint_velocities(gain*(target_joint_positions-rdi.get_joint_positions()));

        rclcpp::spin_some(node);



    PositionControl: ---> Use this mode when you command the robot using ROS 2 commands on the terminal.
                    In this mode, you need to set the target position only. An internal QP computes the velocity commands,
                    taking into account the configuration and configuration velocity limits

*/


class RobotDriverUnitreeZ1: public RobotDriver
{
private:
    RobotDriverUnitreeZ1Configuration configuration_;

    class Impl;
    std::unique_ptr<Impl> impl_;

public:

    RobotDriverUnitreeZ1(const RobotDriverUnitreeZ1&)=delete;
    RobotDriverUnitreeZ1()=delete;
    ~RobotDriverUnitreeZ1();

    RobotDriverUnitreeZ1(const RobotDriverUnitreeZ1Configuration &configuration,
                         std::atomic_bool* break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    void set_target_joint_velocities(const VectorXd& desired_joint_velocities_rad_s) override;

    VectorXd get_joint_velocities() override;
    VectorXd get_joint_torques() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;
};



}
