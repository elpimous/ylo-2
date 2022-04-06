#ifndef PCAN_TO_MOTEUS_HPP
#define PCAN_TO_MOTEUS_HPP

#include <iostream>
#include <string.h> 
//#include <mutex>
#include "PCANDevice.h"

#define MSGTX_ADDR_POSITION 0x06
#define MSGTX_ADDR_VELOCITY 0x0A
#define MSGTX_ADDR_FFTORQUE 0x0E
#define MSGTX_ADDR_KP_SCALE 0x12
#define MSGTX_ADDR_KD_SCALE 0x16
#define MSGTX_ADDR_MAXTORQU 0x1A

#define MSGRX_ADDR_POSITION 0x02
#define MSGRX_ADDR_VELOCITY 0x06
#define MSGRX_ADDR_TORQUE   0x0A

class PCANTOMOTEUS{
public:
    PCANTOMOTEUS(uint32_t id, PCANDevice* can_device_ptr);

    ~PCANTOMOTEUS();

    void send_commands(float fftorque);

    void get_feedback(float& position, float& velocity, float& torque);

    void set_torque_ena(bool torque_ena_);

private:
    // CAN
    uint32_t _id;
    PCANDevice* _can_device_ptr;
    CANDevice::CAN_msg_t _tx_msg_stop;
    CANDevice::CAN_msg_t _tx_msg;
    CANDevice::CAN_msg_t _rx_msg;

    bool _initialized;
    int _fail_count;
    std::string _interface;
    PCANDevice _can_device;

    // CAN Configuration
    CANDevice::Config_t _can_config;

    // Registers
    //std::mutex _torque_ena_mutex;
    bool _torque_ena;
    //std::mutex _command_mutex;
    // values to send
    float _comm_position;
    float _comm_velocity;
    float _comm_fftorque;
    float _comm_kp_scale;
    float _comm_kd_scale;
    float _comm_maxtorqu;
    //std::mutex _feedback_mutex;
    // values to query
    float _position;
    float _velocity;
    float _torque;
};

PCANTOMOTEUS::MoteusPcanInterface(const std::string& port, const int& id)
  : _initialized(false)
  , _running(false)
  , _fail_count(0)
  , _freq_counter(0)
{
  // CAN Configuration
  _can_config.bitrate = 1e6; //1mbps
  _can_config.d_bitrate = 2e6; //2mbps
  _can_config.sample_point = .875; //87.5%
  _can_config.d_sample_point = 0.8; //60%
  _can_config.clock_freq = 80e6; // 80mhz // Read from driver?
  _can_config.mode_fd = 1; // FD Mode
  
  if(!_can_device.Open(interface, _can_config, false))
  {
    return;
  }
  _can_device.ClearFilters();

  // Everything ok
  _initialized = true;
  return;
}

#endif