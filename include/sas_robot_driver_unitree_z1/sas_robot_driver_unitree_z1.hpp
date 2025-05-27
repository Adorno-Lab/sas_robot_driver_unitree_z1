/*
# Copyright (c) 2024 Juan Jose Quiroz Omana
#
#    This file is part of sas_robot_driver_unitree_z1.
#
#    sas_robot_driver_kuka is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_kuka is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_kuka.  If not, see <https://www.gnu.org/licenses/>.
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
    std::string gripper_attached;  //const std::string gripper_attached = "true";
    std::string mode;              //const std::string mode= "PositionControl";
    std::string verbosity;         //const std::string verbosity = "true"
    std::tuple<VectorXd,VectorXd> joint_limits;
};


class RobotDriverUnitreeZ1: public RobotDriver
{
    private:
    RobotDriverUnitreeZ1Configuration configuration_;

    //Implementation details that depend on FRI source files.
    class Impl;
    std::unique_ptr<Impl> impl_;

    public:

    RobotDriverUnitreeZ1(const RobotDriverUnitreeZ1&)=delete;
    RobotDriverUnitreeZ1()=delete;
    ~RobotDriverUnitreeZ1();

    RobotDriverUnitreeZ1(const RobotDriverUnitreeZ1Configuration &configuration, std::atomic_bool* break_loops);

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
