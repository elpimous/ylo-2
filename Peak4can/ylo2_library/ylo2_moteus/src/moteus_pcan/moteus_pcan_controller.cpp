#include "moteus_pcan/moteus_pcan_controller.h" 
#include "moteus_pcan/utils.h"

MoteusPcanController::MoteusPcanController(const MoteusInterfaceMotorsMap& interface_motors_map)
    : _initialized(false)
    , _interface_motors_map(interface_motors_map)
{
    for (auto const& [interface, ids] : interface_motors_map)
    {
        _interfaces.push_back(std::make_shared<MoteusPcanInterface>(interface, ids));
        if(!_interfaces.back()->is_initialized()){
            std::cerr << "[ERROR]: " << "Unable to initialize CAN interface: " << interface << std::endl;
            return;
        }
        for(const auto& [id, motor]: _interfaces.back()->_motors){
            if(!KEY_IN_MAP(id, _motors)){
                _motors[id] = motor;
            }else{
                std::cerr << "[ERROR]: " << "Mutiple definition of ID: " << id << std::endl;
                return;
            }
        }
    }
    _initialized = true;
    return;
}

MoteusPcanController::~MoteusPcanController(){
    for(const auto& interface: _interfaces){
        interface->stop();
    }
}

bool MoteusPcanController::is_initialized(){
    return _initialized;
}

void MoteusPcanController::start(){
    for(const auto& interface: _interfaces){
        interface->start();
    }
}

bool MoteusPcanController::set_torque_ena(bool torque_ena){
    for(const auto& [id, motor]: _motors){
        motor->set_torque_ena(torque_ena);
    }
    return true;
}

bool MoteusPcanController::set_torque_ena(bool torque_ena, int id){
    if(KEY_IN_MAP(id, _motors)){
        _motors[id]->set_torque_ena(torque_ena);
    }else{
        std::cerr << "[ERROR]: " << "ID not defined: " << id << std::endl;
        return false;
    }
    return true;
}

bool MoteusPcanController::set_command(int id, float fftorque){
    if(KEY_IN_MAP(id, _motors)){
        _motors[id]->set_commands(fftorque);
    }else{
        std::cerr << "[ERROR]: " << "ID not defined: " << id << std::endl;
        return false;
    }
    return true;
}

std::vector<int> MoteusPcanController::get_freqs(){
    std::vector<int> freqs;
    for(const auto& interface: _interfaces){
        freqs.push_back(interface->_freq);
    }
    return freqs;
}

bool MoteusPcanController::all_running(){
    for(const auto& interface: _interfaces){
        if(!interface->is_running()){
            return false;
        }
    }
    return true;
}