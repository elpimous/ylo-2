#ifndef MOTEUS_PCAN_INTERFACE_H
#define MOTEUS_PCAN_INTERFACE_H

#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include <atomic>

#include "moteus_pcan/PCANDevice.h"
#include "moteus_pcan/definitions.h"
#include "moteus_pcan/moteus_pcan_motor.h"

typedef std::shared_ptr<MoteusPcanMotor> MoteusPcanMotorPtr;

class MoteusPcanInterface{
public:
    MoteusPcanInterface(const std::string& interface, const std::vector<int>& ids);
    ~MoteusPcanInterface();
    bool is_initialized();
    bool is_running();
    void start();
    void stop();

    std::map<int, MoteusPcanMotorPtr> _motors;
    int _freq;
private:
    bool _initialized;
    std::atomic<bool> _running;
    int _fail_count;
    std::string _interface;
    PCANDevice _can_device;
    CANDevice::Config_t _can_config;
    // std::share_ptr : c'est un pointeur intelligent qui conserve la propriété partagée d'un objet via un pointeur.
    std::shared_ptr<std::thread> _loop_thread; 
    std::shared_ptr<std::thread> _status_loop_thread;
    std::atomic<int> _freq_counter;

    void loop();
    void status_loop();
};

#endif
