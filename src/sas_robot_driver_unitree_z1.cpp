#include "sas_robot_driver_unitree_z1/sas_robot_driver_unitree_z1.hpp"

#include "DriverUnitreeZ1.hpp"
#include <memory>
#include <sas_core/eigen3_std_conversions.hpp>


namespace sas
{

class RobotDriverUnitreeZ1::Impl
{

public:
    std::shared_ptr<DriverUnitreeZ1> unitree_z1_driver_;
    Impl()
    {

    };



};

RobotDriverUnitreeZ1::~RobotDriverUnitreeZ1()
{

}

RobotDriverUnitreeZ1::RobotDriverUnitreeZ1(const RobotDriverUnitreeZ1Configuration &configuration, std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    configuration_(configuration)
{
    impl_ = std::make_unique<RobotDriverUnitreeZ1::Impl>();

    // Extract the driver mode from configuration.mode
    DriverUnitreeZ1::MODE mode;
    if (configuration.mode == "PositionControl")
        mode = DriverUnitreeZ1::MODE::PositionControl;
    else if(configuration.mode == "RawPositionControl")
        mode = DriverUnitreeZ1::MODE::RawPositionControl;
    else
        throw std::runtime_error("The PositionControl or RawPositionControl modes are the only ones supported!");


    impl_->unitree_z1_driver_ = std::make_shared<DriverUnitreeZ1>(break_loops,
                                                                  mode,
                                                                  configuration.gripper_attached,
                                                                  configuration.verbosity,
                                                                  configuration.open_loop_joint_control_gain);
    joint_limits_ = configuration.joint_limits;
}


/**
 * @brief RobotDriverUnitreeZ1::get_joint_positions returns the joint positions including the gripper.
 * @return A vector 7x1 containing the joint positions and the gripper position.
 */
VectorXd RobotDriverUnitreeZ1::get_joint_positions()
{
    return impl_->unitree_z1_driver_->get_joint_positions_with_gripper();
}

/**
 * @brief RobotDriverUnitreeZ1::set_target_joint_positions sets the target joint positions and the target gripper position
 *                  when the operation mode is PositionControl or RawPositionControl.
 * @param desired_joint_positions_rad A vector of 7x1 containing the desired joint positions including the gripper in radians.
 */
void RobotDriverUnitreeZ1::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    impl_->unitree_z1_driver_->set_target_joint_positions_with_gripper(desired_joint_positions_rad);
}

/**
 * @brief RobotDriverUnitreeZ1::set_target_joint_velocities sets the target joint velocities and the target gripper velocity
 * @param desired_joint_velocities_rad_s A vector of 7x1 containing the desired joint velocities (including the gripper).
 */
void RobotDriverUnitreeZ1::set_target_joint_velocities(const VectorXd &desired_joint_velocities_rad_s)
{
    impl_->unitree_z1_driver_->set_target_joint_velocities_with_gripper(desired_joint_velocities_rad_s);
}

/**
 * @brief RobotDriverUnitreeZ1::get_joint_velocities returns the measured joint velocities.
 * @return a vector containing the joint velocities. This vector is 7x1 and includes the gripper velocity.
 */
VectorXd RobotDriverUnitreeZ1::get_joint_velocities()
{
    return impl_->unitree_z1_driver_->get_joint_velocities_with_gripper();
}

/**
 * @brief RobotDriverUnitreeZ1::get_joint_torques  returns the measured joint torques.
 * @return a vector containing the joint torques. This vector is 7x1 and includes the gripper effort.
 */
VectorXd RobotDriverUnitreeZ1::get_joint_torques()
{
    return impl_->unitree_z1_driver_->get_joint_torques_with_gripper();
}


/**
 * @brief RobotDriverUnitreeZ1::connect
 */
void RobotDriverUnitreeZ1::connect()
{
    impl_->unitree_z1_driver_->connect();
}


/**
 * @brief RobotDriverUnitreeZ1::disconnect
 */
void RobotDriverUnitreeZ1::disconnect()
{
    impl_->unitree_z1_driver_->disconnect();
}


/**
 * @brief RobotDriverUnitreeZ1::initialize
 */
void RobotDriverUnitreeZ1::initialize()
{
    impl_->unitree_z1_driver_->move_to_initial_configuration_when_initialized(configuration_.move_to_initial_configuration,
                                                                              configuration_.initial_configuration);
    impl_->unitree_z1_driver_->initialize();
}

/**
 * @brief RobotDriverUnitreeZ1::deinitialize
 */
void RobotDriverUnitreeZ1::deinitialize()
{
    impl_->unitree_z1_driver_->deinitialize();
}




}
