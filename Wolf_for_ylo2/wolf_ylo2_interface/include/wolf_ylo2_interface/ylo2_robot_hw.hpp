#ifndef ylo2_ROBOT_HW_H
#define ylo2_ROBOT_HW_H

#include <wolf_hardware_interface/wolf_robot_hw.h>
//#include <ylo2_moteus_controller/moteus_controller.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ylo22ros
{

class ylo2RobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  ylo2RobotHw();
  virtual ~ylo2RobotHw();

  void init();
  void read();
  void write();

private:

  /** @brief Map ylo2 internal joint indices to WoLF joints order */
  std::array<unsigned int, 12> ylo2_motor_idxs_
          {{
          ylo2hal::FL_0, ylo2hal::FL_1, ylo2hal::FL_2, // LF
          ylo2hal::RL_0, ylo2hal::RL_1, ylo2hal::RL_2, // LH
          ylo2hal::FR_0, ylo2hal::FR_1, ylo2hal::FR_2, // RF
          ylo2hal::RR_0, ylo2hal::RR_1, ylo2hal::RR_2, // RH
          }};

  /** @brief ylo2-HAL */
  ylo2hal::LowLevelInterface ylo2_interface_;
  ylo2hal::LowState ylo2_state_ = {0};
  ylo2hal::LowCmd ylo2_lowcmd_ = {0};

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();

};

}

#endif
