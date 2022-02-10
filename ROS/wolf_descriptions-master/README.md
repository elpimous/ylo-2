# WoLF: Whole-body Locomotion Framework for quadruped robots

This repo contains a collection of different robots and sensors used in WoLF.

## Setup

See the documentation [here](https://github.com/graiola/wolf-setup/blob/master/README.md).

## How to add a new robot

To add a new robot you can start by copying the structure of an existing one. The robot descriptions are defined as ROS packages, so you will need to add it to your ROS workspace.
The required files in the package are the following:

- `params/robot_name_params.yaml` it contains the controller parameters, use this file to tune a robot.
- `robots/robot_name.urdf.xacro` it is the robot URDF description describing the kinematics and dynamics of the robot.
- `robots/robot_name.srdf.xacro` it is the robot SRDF description, use this file to define the kinematic chains and default postures.

You can add plotjuggler and rviz visualization files in the respective folders `plotjuggler` and `rviz`. The folder `urdfs` can be used to store extra xacro files useful for the robot description. And last but not least, don't forget the robot's meshes in the `meshes` folder, unless your robot is a stealth one :)

To launch the new robot with the WoLF controller:

`roslaunch wolf_controller wolf_controller_bringup.launch robot_name:=new_robot`

## Parameters description

- `default_duty_factor` defines the ratio between the stance and total cycle time: `T_stance / T_cycle`.
- `default_swing_frequency` defines the frequency of the swing: `1 / T_swing`.
- `default_contact_threshold` defines the force magnitude  that is used to detect if the robot is contact with the enviroment or not.
- `default_step_reflex_contact_threshold` if the step reflex is active, it represents the magnitude of the contact force that triggers a step reflex.
- `default_base_linear_velocity_x` linear velocity along robot's x axis.
- `default_base_linear_velocity_y` linear velocity along robot's y axis.
- `default_base_linear_velocity_z` linear velocity along robot's z axis.
- `default_base_linear_velocity` defines a common velocity value along xyz axis, it overrides the previous linear velocities.
- `default_base_angular_velocity_roll`  angular velocity along robot's roll axis.
- `default_base_angular_velocity_pitch`  angular velocity along robot's pitch axis.
- `default_base_angular_velocity_yaw`  angular velocity along robot's yaw axis.
- `default_base_angular_velocity` defines a common velocity value along roll pitch and yaw axis, it overrides the previous angular velocities.
- `default_step_height` defines the step height.
- `default_cutoff_freq_gyroscope` cut off frequency for the gyroscope filter.
- `default_cutoff_freq_qdot` cut off frequency for the joint velocities filter.
- `default_friction_cones_mu` friction cones [mu](https://scaron.info/robot-locomotion/friction-cones.html).
- `max_step_height` defines the maximum step height allowed.
- `max_step_length` defines the maximum step length allowed.
- `max_base_height` defines the maximum base height allowed.
- `max_base_roll` defines the maximum rotation along the robot's roll axis.
- `min_base_roll` defines the minimum rotation along the robot's roll axis.
- `max_base_pitch` defines the maximum rotation along the robot's pitch axis. 
- `min_base_pitch` defines the minimum rotation along the robot's pitch axis.
- `estimation_position_type [ground_truth (simulation only) | estimated_z]` 
- `estimation_orientation_type [ground_truth (simulation only) | imu_magnetometer | imu_gyroscope]` 
- `activate_step_reflex [true | false]` activate the step reflex.
- `activate_push_recovery [true | false]` activate the push recovery .
- `initial_pose_simulation` Initial robot's pose in simulation.

```
gains:

      Kp_leg: {haa: xxx, hfe: xxx, kfe:xxx}
      Kd_leg: {haa: xxx, hfe: xxx, kfe: xxx}

      xx_foot:
          Kp: {x: xxx, y: xxx, z: xxx, roll: 0, pitch: 0, yaw: 0}
          Kd: {x: xxx, y: xxx, z: xxx, roll: 0, pitch: 0, yaw: 0}
          type: [force|acceleration]
          weight: xxx

      waistRPY:
          Kp: {x: 0.0, y: 0.0, z: 0.0, roll: xxx, pitch: xxx, yaw: xxx}
          Kd: {x: 0.0, y: 0.0, z: 0.0, roll: xxx, pitch: xxx, yaw: xxx}
          type: [force|acceleration]
          weight: xxx

      CoM:
          Kp: {x: xxx, y: xxx z: xxx}
          Kd: {x: xxx, y: xxx, z: xxx}
          weight: xxx

      angular_momentum:
          weight: xxx
```

- `Kp_leg, Kd_leg` define the impedance gains for the joints in each leg. These gains are  in two istances: to zero the joints during the init phase and in case of anomaly to perform a controlled shutdown (in this istance, only the damping is used).
- tasks:
	- `Kp,Kd` task's gains.
	- `type  [force|acceleration]` by default the tasks' gains are defined as force gains meaning that `Kp` and `Kd` work as Cartesian impedance gains. Please check Appendix A in the following [paper](https://hal.archives-ouvertes.fr/hal-03005133/document) for a better explanation of Cartesian impedance.
	- `weight` relative task weight compared to the other tasks. A higher value means that the selected task has a higher priority and therefore the solver will try to minimize this task first.

## How to reset the robot pose and posture

To reset the robot pose in gazebo you can simply use the gazebo's shortcut `ctrl-r`, this command should set the robot pose to the origin. Then, to reset the robot's posture run the following:

`rosrun wolf_description_utils go0`

now your robot should be ready to stand up again!
