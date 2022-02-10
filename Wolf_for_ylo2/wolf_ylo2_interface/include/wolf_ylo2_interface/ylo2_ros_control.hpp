#ifndef ylo2_ROS_CONTROL_H
#define ylo2_ROS_CONTROL_H

#include <memory>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "wolf_ylo2_interface/ylo2_robot_hw.hpp"


namespace ylo22ros {

class ylo2ROSControl {
public:

    std::string CLASS_NAME = "ylo2ROSControl";

	ylo2ROSControl();
	~ylo2ROSControl();

  /** @brief init */
	void init();

  /** @brief update */
	void update(const ros::Time& time, const ros::Duration& period);

private:

  /** @brief ROS node handle */
	std::shared_ptr<ros::NodeHandle> node_handle_;
	
  /** @brief ylo2 Hardware interface */
	std::shared_ptr<ylo2RobotHw> robot_hw_;

  /** @brief controller_manager provides the infrastructure to load, unload, start and stop controllers */
	std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

} // namespace


#endif
