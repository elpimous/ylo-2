# ROS2 Packages for K3lso v2 Quadruped Robot

**K3lso moteus controllers package**

Based on (find the original moteus lib)

## Dependencies

Install pcandriver and pcanbasic: https://github.com/raess1/K3lso-CAN-communication

## Executions

* **Main Moteus interface node**:

``` bash
ros2 run k3lso_moteus k3lso_moteus_node
```

## Services

Enable torque:

``` bash
ros2 service call /k3lso_moteus/set_torque k3lso_msgs/srv/MotorsSetTorque "{ids: [5, 6], joint_names: [], state: [false, true]}"
```

