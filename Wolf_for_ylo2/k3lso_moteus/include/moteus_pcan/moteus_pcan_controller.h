#ifndef MOTEUS_PCAN_CONTROLLER_H
#define MOTEUS_PCAN_CONTROLLER_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "moteus_pcan/definitions.h"
#include "moteus_pcan/moteus_pcan_interface.h"

typedef std::shared_ptr<MoteusPcanInterface> MoteusPcanInterfacePtr;
typedef std::map<std::string, std::vector<int>> MoteusInterfaceMotorsMap;

class MoteusPcanController{
public:
    MoteusPcanController(const MoteusInterfaceMotorsMap& interface_motors_map);
    ~MoteusPcanController();
    bool is_initialized();
    void start();
    bool all_running();

    bool set_torque_ena(bool torque_ena);
    bool set_torque_ena(bool torque_ena, int id);
    bool set_torque_ena(bool torque_ena, std::vector<int> ids);

    // send to individual controller ID
    bool set_command(int id, float position, float velocity, float fftorque, float kp_scale, float kd_scale);
    bool set_command(int id, float position, float velocity, float fftorque);
    bool set_command(int id, float position, float velocity);
    bool set_command(int id, float position);

    // Send to multiple controllers ID
    bool set_command(std::vector<int> ids, std::vector<float> position, std::vector<float> velocity, std::vector<float> fftorque, 
                     std::vector<float> kp_scale, std::vector<float> kd_scale);
    bool set_command(std::vector<int> ids, std::vector<float> position, std::vector<float> velocity, std::vector<float> fftorque);
    bool set_command(std::vector<int> ids, std::vector<float> position, std::vector<float> velocity);
    bool set_command(std::vector<int> ids, std::vector<float> position);

    std::vector<int> get_freqs();
    std::map<int, MoteusPcanMotorPtr> _motors; // Volver privado de nuevo
private:
    bool _initialized;
    MoteusInterfaceMotorsMap _interface_motors_map;
    std::vector<MoteusPcanInterfacePtr> _interfaces;
};

#endif