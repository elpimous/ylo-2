#include "wolf_ylo2_interface/ylo2_robot_hw.hpp"

namespace ylo22ros
{

using namespace hardware_interface;

int64_t utime_now() {

    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    if (timeofday.tv_sec < 0 || timeofday.tv_sec > UINT_MAX)
        throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
    uint32_t sec	= timeofday.tv_sec;
    uint32_t nsec = timeofday.tv_usec * 1000;

    return (int64_t) (((uint64_t)sec)*1000000 + ((uint64_t)nsec) / 1000);
}

ylo2RobotHw::ylo2RobotHw()
{
    robot_name_ = "ylo2";
}

ylo2RobotHw::~ylo2RobotHw()
{

}

void ylo2RobotHw::init()
{
    // Hardware interfaces: Joints
    auto joint_names = loadJointNamesFromSRDF();
    if(joint_names.size()>0)
    {
      WolfRobotHwInterface::initializeJointsInterface(joint_names);
      registerInterface(&joint_state_interface_);
      registerInterface(&joint_effort_interface_);
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register joint interface.");
      return;
    }

    // Hardware interfaces: IMU
    auto imu_name = loadImuLinkNameFromSRDF();
    if(!imu_name.empty())
    {
      WolfRobotHwInterface::initializeImuInterface(imu_name);
      registerInterface(&imu_sensor_interface_);
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register imu interface.");
      return;
    }

    ylo2_interface_.InitCmdData(ylo2_lowcmd_);
    startup_routine();
}

void ylo2RobotHw::read()
{
    // Get robot data
    ylo2_state_ = ylo2_interface_.ReceiveObservation();

    // ------
    // Joints
    // ------
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
    {
        joint_position_[jj] = static_cast<double>(ylo2_state_.motorState[ylo2_motor_idxs_[jj]].q)     ;
        joint_velocity_[jj] = static_cast<double>(ylo2_state_.motorState[ylo2_motor_idxs_[jj]].dq)    ;
        joint_effort_[jj]   = static_cast<double>(ylo2_state_.motorState[ylo2_motor_idxs_[jj]].tauEst);
    }

    // ---
    // IMU
    // ---
    imu_orientation_[0] = static_cast<double>(ylo2_state_.imu.quaternion[0]);  // w
    imu_orientation_[1] = static_cast<double>(ylo2_state_.imu.quaternion[1]);  // x
    imu_orientation_[2] = static_cast<double>(ylo2_state_.imu.quaternion[2]);  // y
    imu_orientation_[3] = static_cast<double>(ylo2_state_.imu.quaternion[3]);  // z

    imu_ang_vel_[0] = static_cast<double>(ylo2_state_.imu.gyroscope[0]);
    imu_ang_vel_[1] = static_cast<double>(ylo2_state_.imu.gyroscope[1]);
    imu_ang_vel_[2] = static_cast<double>(ylo2_state_.imu.gyroscope[2]);

    imu_lin_acc_[0] = static_cast<double>(ylo2_state_.imu.accelerometer[0]);
    imu_lin_acc_[1] = static_cast<double>(ylo2_state_.imu.accelerometer[1]);
    imu_lin_acc_[2] = static_cast<double>(ylo2_state_.imu.accelerometer[2]);
}

void ylo2RobotHw::write()
{
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
      ylo2_lowcmd_.motorCmd[ylo2_motor_idxs_[jj]].tau = static_cast<float>(joint_effort_command_[jj]  );

    ylo2_interface_.SendLowCmd(ylo2_lowcmd_);
}

void ylo2RobotHw::send_zero_command()
{
    std::array<float, 60> zero_command = {0};
    // ylo2_interface_->SendCommand(zero_command);
    ylo2_interface_.SendCommand(zero_command);
}

void ylo2RobotHw::startup_routine()
{
    send_zero_command();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

} // namespace
