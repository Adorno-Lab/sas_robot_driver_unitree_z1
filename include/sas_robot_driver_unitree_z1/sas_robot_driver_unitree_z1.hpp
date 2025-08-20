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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace rclcpp;
using namespace Eigen;

namespace sas
{

struct RobotDriverUnitreeZ1Configuration
{
    bool gripper_attached;  //const bool gripper_attached = = true;
    std::string mode;       //const std::string mode= "PositionControl";
    bool verbosity;         //const bool verbosity = true;
    std::tuple<VectorXd,VectorXd> joint_limits;
    std::string robot_name;
};


class RobotDriverUnitreeZ1: public RobotDriver
{
    private:
    RobotDriverUnitreeZ1Configuration configuration_;

    std::shared_ptr<rclcpp::Node> node_;
    std::string topic_prefix_;

    //  For custom commands
    bool new_target_velocities_available_{false};
    VectorXd target_raw_commands_ = VectorXd::Zero(13);// [6--> joint positions + 6--> joint velocities + 1-->gripper position]
    void _callback_target_joint_raw_commands(const std_msgs::msg::Float64MultiArray& msg);
    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_target_joint_raw_commands_;


    //Implementation details that depend on FRI source files.
    class Impl;
    std::unique_ptr<Impl> impl_;

    public:

    RobotDriverUnitreeZ1(const RobotDriverUnitreeZ1&)=delete;
    RobotDriverUnitreeZ1()=delete;
    ~RobotDriverUnitreeZ1();

    RobotDriverUnitreeZ1(std::shared_ptr<Node>& node,
                         const RobotDriverUnitreeZ1Configuration &configuration,
                         std::atomic_bool* break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    VectorXd get_joint_velocities() override;
    //void set_target_joint_velocities(const VectorXd& desired_joint_velocities_rads) override; //Not possible (yet?)

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;
};



}
