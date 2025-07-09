#include "DriverUnitreeZ1.hpp"
#include "unitree_arm_sdk/control/unitreeArm.h"
#include "constraints_manager.hpp"

class DriverUnitreeZ1::Impl
{
public:
    std::shared_ptr<UNITREE_ARM::unitreeArm> arm_;
    std::shared_ptr<UNITREE_ARM::Timer> unitree_timer_;
    std::shared_ptr<UNITREE_ARM::CtrlComponents> ctrlComp_;
    std::unique_ptr<Capybara::ConstraintsManager> cm_;
    Impl()
    {

    };
};


/**
 * @brief DriverUnitreeZ1::DriverUnitreeZ1 constructor of the class.
 * @param st_break_loops Use this flag to break internal loops. This is useful to stop the robot using a signal
 *              interruption by the user.
 * @param mode The operation mode. Select the control strategy to command the robot.
 * @param gripper_attached Use true (default) if the robot is equipped with the Unitree Gripper. Use false otherwise.
 * @param verbosity. Use true (default) to display more information in the terminal.
 *
 *
 * Example:
 *          // This class follows the SmartArmStack driver principles, in which four methods are required to
 *             start and finish the robot communication.
 *
 *          DriverUnitreeZ1 Z1(&kill_this_process, DriverUnitreeZ1::MODE::PositionControl);
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
DriverUnitreeZ1::DriverUnitreeZ1(std::atomic_bool *st_break_loops, const MODE &mode,
                                           const bool &gripper_attached,
                                           const bool& verbosity)
    :st_break_loops_{st_break_loops}, mode_{mode}, verbosity_{verbosity},
    gripper_attached_{gripper_attached}, finish_echo_robot_state_{false}
{

    impl_       = std::make_shared<DriverUnitreeZ1::Impl>();

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

    H_  = MatrixXd::Identity(6, 6);
    I_  = MatrixXd::Identity(6, 6);
    impl_->cm_ = std::make_unique<Capybara::ConstraintsManager>(6);
    solver_ = std::make_unique<DQ_QPOASESSolver>();

    _set_driver_mode(mode);
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
    VectorXd q_dot_min = -q_dot_max_;
    impl_->cm_->add_inequality_constraint(-I_, -nq*(-(q-q_min_)));
    impl_->cm_->add_inequality_constraint( I_, -nq*( (q-q_max_)));
    impl_->cm_->add_inequality_constraint(-I_,  -q_dot_min);
    impl_->cm_->add_inequality_constraint( I_,   q_dot_max_);
    MatrixXd A;
    VectorXd b;
    auto inequality_constraints = impl_->cm_->get_inequality_constraints();
    A = std::get<0>(inequality_constraints);
    b = std::get<1>(inequality_constraints);
    //auto [A, b] = impl_->cm_->get_inequality_constraints(); C++17
    VectorXd u = solver_->solve_quadratic_program(H_, f, A, b, Aeq, beq);
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
 * @brief DriverUnitreeZ1::_update_q_for_numerical_integration updates the robot state including the gripper position.
 *                          This method must be called before to start the joint position control loop.
 */
void DriverUnitreeZ1::_update_q_for_numerical_integration()
{
    for (int i=0;i<100;i++) // Updated the robot configuration
    {
        _update_robot_state();
        q_ni_ = q_measured_;
        q_gripper_ni_ = gripper_position_measured_;
        impl_->unitree_timer_->sleep();
    }
}

void DriverUnitreeZ1::_set_driver_mode(const MODE &mode)
{
    switch (mode){

    case MODE::None:
        mode_ = mode;
        break;
    case MODE::PositionControl:
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
    q_measured_     = impl_->arm_->lowstate->getQ();
    q_dot_measured_ = impl_->arm_->lowstate->getQd();
    q_dot_dot_measured_ = impl_->arm_->lowstate->getQdd();
    tau_measured_  = impl_->arm_->lowstate->getTau();

    std::vector<int> temp = impl_->arm_->lowstate->temperature;
    motor_temperatures_ = Eigen::Map<VectorXi>(temp.data(), temp.size());

    gripper_position_measured_ = impl_->arm_->lowstate->getGripperQ();
    gripper_velocity_measured_ = impl_->arm_->lowstate->getGripperQd();
    /*
     * 0x01 : phase current is too large
     * 0x02 : phase leakage
     * 0x04 : motor winding overheat or temperature is too large
     * 0x20 : parameters jump
     * 0x40 : Ignore
     */
    errorstate_ = impl_->arm_->lowstate->errorstate;
}

// This name does not make sense. I'm going to change it to echo_robot_state_mode().
void DriverUnitreeZ1::_echo_robot_state_mode()
{
    while(!finish_echo_robot_state_ or !st_break_loops_)
    {
        _update_robot_state();
        impl_->unitree_timer_->sleep();
    }
    status_msg_ = "Echo robot state finished.";
    _show_status();
}


/**
 * @brief DriverUnitreeZ1::_start_echo_robot_state_mode_thread This method starts the echo loop thread that updates
 *                      the robot state. This loop however, can not command the robot. It is only for receive information
 *                      from the robot.
 */
void DriverUnitreeZ1::_start_echo_robot_state_mode_thread()
{
    finish_echo_robot_state_ = false;
    if (echo_robot_state_mode_thread_.joinable())
    {
        echo_robot_state_mode_thread_.join();
    }
    echo_robot_state_mode_thread_ = std::thread(&DriverUnitreeZ1::_echo_robot_state_mode, this);
}

/**
 * @brief DriverUnitreeZ1::_finish_echo_robot_state this method finishes the echo loop thread.
 */
void DriverUnitreeZ1::_finish_echo_robot_state()
{
    status_msg_ = "Finishing echo robot state.";
    finish_echo_robot_state_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    _show_status();
}


void DriverUnitreeZ1::_joint_position_control_mode()
{
    _update_q_for_numerical_integration();
    target_joint_positions_ = q_ni_;
    target_gripper_position_ = q_gripper_ni_;
    VectorXd q_dot;
    double q_gripper_position_dot;
    double gain_gripper = 1.0;
    while(!finish_motion_ or !st_break_loops_)
    {
        _update_robot_state();
        auto results = _compute_control_inputs(q_ni_, target_joint_positions_, 5.0);
        q_ni_    = std::get<0>(results);
        q_dot = std::get<1>(results);

        impl_->arm_->q = q_ni_;
        impl_->arm_->qd = q_dot;

        impl_->arm_->setArmCmd(impl_->arm_->q, impl_->arm_->qd);

        if (gripper_attached_)
        {
            q_gripper_position_dot = -gain_gripper*(q_gripper_ni_-target_gripper_position_);
            q_gripper_ni_ = q_gripper_ni_ + T_*q_gripper_position_dot;
            impl_->arm_->setGripperCmd(q_gripper_ni_ , q_gripper_position_dot);
        }

        impl_->unitree_timer_->sleep();
    }
    status_msg_ = "Joint position control finished.";
    _show_status();

}


/**
 * @brief DriverUnitreeZ1::_start_joint_position_control_thread This method starts the control loop thread that updates
 *                      the robot state and control the robot.
 */
void DriverUnitreeZ1::_start_joint_position_control_thread()
{
    finish_motion_ = false;
    if (joint_position_control_mode_thread_.joinable())
    {
        joint_position_control_mode_thread_.join();
    }
    joint_position_control_mode_thread_ = std::thread(&DriverUnitreeZ1::_joint_position_control_mode, this);
    status_msg_ = "Starting joint position control.";
    _show_status();
}


/**
 * @brief DriverUnitreeZ1::connect This method establishes the connection with robot. This is the first method to be called
 *                      after the constructor of the class.
 */
void DriverUnitreeZ1::connect()
{
    if (current_status_ == STATUS::IDLE)
    {
        //std::cout<<impl_->z1_controller_->_get_config_path()<<std::endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));

        impl_->arm_->sendRecvThread->start();
        T_ = impl_->arm_->_ctrlComp->dt;

        if (!impl_->unitree_timer_)
            impl_->unitree_timer_ = std::make_shared<UNITREE_ARM::Timer>(T_);

        _start_echo_robot_state_mode_thread();
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
        impl_->arm_->backToStart();
        impl_->arm_->startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);

        std::cout<<"Reading robot configuration..."<<std::endl;
        _update_q_for_numerical_integration();


        initial_robot_configuration_ = q_ni_;

        if (move_robot_to_forward_position_when_initialized_)
        {
            std::cout<<"Setting forward robot configuration..."<<std::endl;
            _move_robot_to_target_joint_positions(Forward_, 0.3, st_break_loops_);
        }


        switch (mode_) {
        case MODE::None:
            break;
        case MODE::PositionControl:
            _finish_echo_robot_state();
            _start_joint_position_control_thread();
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
 * @brief DriverUnitreeZ1::deinitialize This method stops all communication threads.
 *                  This method requires initialized communication with the robot, i.e. the user must call both connect(),
 *                  and initialize() before calling deinitialize().
 */
void DriverUnitreeZ1::deinitialize()
{

    _finish_motion();

    while(!finish_motion_){
        std::cout<<"Waiting..."<<std::endl;
    }; //wait to break the joint control thread

    // Force this loop to keep enabled to move the robot to its initial configuration
    std::atomic_bool flag{false};
    std::cout<<"Setting home robot configuration..."<<std::endl;
    _move_robot_to_target_joint_positions(initial_robot_configuration_, 0.3, &flag);

    impl_->arm_->backToStart();
    impl_->arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    impl_->arm_->sendRecvThread->shutdown();
    status_msg_ = "Deinitialized.";
    current_status_ = STATUS::DEINITIALIZED;
    _show_status();
}


/**
 * @brief DriverUnitreeZ1::disconnect
 */
void DriverUnitreeZ1::disconnect()
{
    status_msg_ = "Disconnecting...";
    _finish_echo_robot_state();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (joint_position_control_mode_thread_.joinable())
         joint_position_control_mode_thread_.join();

    if (echo_robot_state_mode_thread_.joinable())
        echo_robot_state_mode_thread_.join();

    status_msg_ = "Disconnected.";
    current_status_ = STATUS::DISCONNECTED;
    _show_status();
    //impl_->z1_controller_->stop();
}

void DriverUnitreeZ1::set_target_gripper_position(const double &target_gripper_position)
{
    target_gripper_position_ = target_gripper_position;
}

void DriverUnitreeZ1::_finish_motion()
{
    std::cout<<"Ending control loop..."<<std::endl;
    for (int i=0;i<10;i++)
    {
        set_target_joint_positions(q_ni_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    finish_motion_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}


/**
 * @brief DriverUnitreeZ1::_move_robot_to_target_joint_positions commands the robot in joint position level.
 *                      This method is used in the joint position control thread.
 *
 * @param q_target   The desired robot configuration.
 * @param gain       Convergence factor
 * @param break_loop Flag to break the loop.
 */
void DriverUnitreeZ1::_move_robot_to_target_joint_positions(const VectorXd &q_target,
                                                            const double &gain,
                                                            std::atomic_bool *break_loop)
{
    _update_q_for_numerical_integration();
    VectorXd q_dot;
    while ( ((q_ni_-q_target).norm() > 0.01) and !(*break_loop))
    {
        _update_robot_state();
        auto results = _compute_control_inputs(q_ni_, q_target, gain);
        q_ni_     = std::get<0>(results);
        q_dot = std::get<1>(results);

        // To move the robot in joint position commands, we need to define both a target robot configuration and a target
        // robot configuration velocity.
        impl_->arm_->q = q_ni_;
        impl_->arm_->qd = q_dot;
        impl_->arm_->setArmCmd(impl_->arm_->q, impl_->arm_->qd);
        impl_->unitree_timer_->sleep();
    }
}

/**
 * @brief DriverUnitreeZ1::get_joint_positions returns the meausured joint positions.
 * @return a vector containing the joint positions. This vector is 6x1 and does not include the gripper position.
 */
VectorXd DriverUnitreeZ1::get_joint_positions()
{
    return q_measured_;
}

/**
 * @brief DriverUnitreeZ1::get_joint_velocities returns the meausured joint velocities.
 * @return a vector containing the joint velocities. This vector is 6x1 and does not include the gripper velocity.
 */
VectorXd DriverUnitreeZ1::get_joint_velocities()
{
    return q_dot_measured_;
}

/**
 * @brief DriverUnitreeZ1::get_joint_velocities_with_gripper returns the meausured joint velocities.
 * @return a vector containing the joint velocities. This vector is 7x1 and includes the gripper velocity.
 */
VectorXd DriverUnitreeZ1::get_joint_velocities_with_gripper()
{
    VectorXd joint_velocities_with_gripper = VectorXd::Zero(7);
    joint_velocities_with_gripper << q_dot_measured_(0), q_dot_measured_(1), q_dot_measured_(2), q_dot_measured_(3), q_dot_measured_(4), q_dot_measured_(5), gripper_velocity_measured_;
    return joint_velocities_with_gripper;
}

/**
 * @brief DriverUnitreeZ1::get_joint_forces returns the estimated joint forces.
 * @return a vector containing the joint forces. This vector is 6x1 and does not include the gripper effort.
 */
VectorXd DriverUnitreeZ1::get_joint_forces()
{
    return tau_measured_;
}

/**
 * @brief DriverUnitreeZ1::get_gripper_position returns the gripper position in radians.
 * @return The gripper position. If the robot is started in the home configuration, A value of zero denotes the gripper closed.
 *         A negative value denotes the gripper opened.
 */
double DriverUnitreeZ1::get_gripper_position()
{
    return gripper_position_measured_;
}


/**
 * @brief DriverUnitreeZ1::move_robot_to_forward_position_when_initialized moves the robot to the forward configuration.
 *                      This method must be called after connect() and before initialize().
 * @param flag Use true if you want to move the robot to the forward configuration before initializing the driver.
 */
void DriverUnitreeZ1::move_robot_to_forward_position_when_initialized(const bool &flag)
{
    move_robot_to_forward_position_when_initialized_ = flag;
}


/**
 * @brief DriverUnitreeZ1::move_robot_to_target_joint_positions moves the robot to a desired robot configuration.
 *                      This method is designed to set a start robot configuration before to start a control loop.
 *                      If the operation mode is None, the method works, but this feature is going to be removed.
 *                      If the operation mode is PositionControl, the driver must be connected but not initialized.
 * @param q_target The desired robot configuration.
 */
void DriverUnitreeZ1::move_robot_to_target_joint_positions(const VectorXd &q_target)
{
    if (current_status_ == STATUS::INITIALIZED && mode_ == MODE::None)
    {
        _move_robot_to_target_joint_positions(q_target, 0.1, st_break_loops_);
    }else if( mode_ == MODE::PositionControl && current_status_ == STATUS::CONNECTED)
    {
        _move_robot_to_target_joint_positions(q_target, 0.1, st_break_loops_);
    }
    else
    {
        std::cerr<<"This method is enabled for MODE::None and STATUS::INITIALIZED"<<std::endl;
        std::cerr<<"Or MODE::PositionControl and STATUS::CONNECTED."<<std::endl;
    }
}

/**
 * @brief DriverUnitreeZ1::set_target_joint_positions sets the target joint positions when the operation mode is PositionControl.
 * @param target_joint_positions_rad The target joint positions in radians.
 */
void DriverUnitreeZ1::set_target_joint_positions(const VectorXd &target_joint_positions_rad)
{
    target_joint_positions_ = target_joint_positions_rad;
}


/**
 * @brief DriverUnitreeZ1::set_gripper_position sets the gripper position when the operation mode is PositionControl.
 * @param gripper_position The target gripper position in radians. If the robot is correctly started following the official instructions,
 *                      a value of zero represents a closed gripper. A value lower than zero is to open the gripper.
 *                      Positive values are ignored.
 */
void DriverUnitreeZ1::set_gripper_position(const double &gripper_position)
{
    target_gripper_position_ = gripper_position;
}

/**
 * @brief DriverUnitreeZ1::set_target_joint_positions_with_gripper sets the target joint positions and the target gripper position
 *                  when the operation mode is PositionControl.
 * @param target_joint_positions_with_gripper_rad A vector of 7x1 containing the desired joint positions including the gripper.
 */
void DriverUnitreeZ1::set_target_joint_positions_with_gripper(const VectorXd &target_joint_positions_with_gripper_rad)
{
    set_target_joint_positions(target_joint_positions_with_gripper_rad.head(6));
    set_gripper_position(target_joint_positions_with_gripper_rad(6));
}

/**
 * @brief DriverUnitreeZ1::get_joint_positions_with_gripper returns the measured joint positions including the gripper.
 * @return A vector 7x1 containing the joint positions and the gripper position.
 */
VectorXd DriverUnitreeZ1::get_joint_positions_with_gripper()
{
    VectorXd joint_positions_with_gripper = VectorXd::Zero(7);
    joint_positions_with_gripper << q_measured_(0), q_measured_(1), q_measured_(2), q_measured_(3), q_measured_(4), q_measured_(5), gripper_position_measured_;
    return joint_positions_with_gripper;
}




