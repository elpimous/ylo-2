#ifndef MOTEUS_PCAN_MOTOR_H
#define MOTEUS_PCAN_MOTOR_H

#include <iostream>
#include <string.h> 
#include <mutex>

#include "moteus_pcan/definitions.h"
#include "moteus_pcan/moteus_pcan_motor.h"

#include "PCANDevice.h"

class MoteusPcanMotor{
public:
    MoteusPcanMotor(uint32_t id, PCANDevice* can_device_ptr);

    ~MoteusPcanMotor();

    bool write_read();

    void set_commands(float fftorque);

    void get_feedback(float& position, float& velocity, float& torque);

    void set_torque_ena(bool torque_ena_);

private:
    // CAN
    uint32_t _id;
    PCANDevice* _can_device_ptr;
    CANDevice::CAN_msg_t _msg_tx_stop;
    CANDevice::CAN_msg_t _tx_msg;
    CANDevice::CAN_msg_t _rx_msg;
    // Registers
    std::mutex _torque_ena_mutex;
    bool _torque_ena;
    std::mutex _command_mutex;
    float _comm_position;
    float _comm_velocity;
    float _comm_fftorque;
    float _comm_kp_scale;
    float _comm_kd_scale;
    float _comm_maxtorqu;
    std::mutex _feedback_mutex;
    float _position;
    float _velocity;
    float _torque;
};

#endif