// File adapted for Ylo2 robot, for pure torque commands

#include "moteus_pcan/moteus_pcan_motor.h"
#include "moteus_pcan/utils.h"

// RX
#define MSGRX_ADDR_POSITION 0x02
#define MSGRX_ADDR_VELOCITY 0x06
#define MSGRX_ADDR_TORQUE   0x0A

using namespace std::chrono_literals;

MoteusPcanMotor::MoteusPcanMotor(uint32_t id, PCANDevice* can_device_ptr)
    : _id(id)
    , _can_device_ptr(can_device_ptr)
    , _torque_ena(false)
{
    
    // TX STOP PACKAGE
    _msg_tx_stop.id = 0x8000 | id; // in moteus lib, for example 0x8002 means 80 = query values, and 02 = ID
    _msg_tx_stop.length = 5;
    // Write Mode
    _msg_tx_stop.data[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
    _msg_tx_stop.data[1] = 0x00; // Register to write: MODE
    _msg_tx_stop.data[2] = 0x00; // Value to write: STOPPED MODE
    // Query
    _msg_tx_stop.data[3] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
    _msg_tx_stop.data[4] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE

    // -----------------------------------------------------------------------------

    // TX POS PACKAGE for Tau mode only (torque, kp=0, kd=0)
    _tx_msg.id = 0x8000 | id;
    _tx_msg.length = 12;  // TODO: less bytes ?
    // Write Mode
    _tx_msg.data[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
    _tx_msg.data[1] = 0x00; // Register to write: MODE
    _tx_msg.data[2] = 0x0A; // Value to write: POSITION MODE
    // Write command
    _tx_msg.data[3] = 0x0C; // Write floats
    _tx_msg.data[4] = 0x01; // 0x01 Write 3 registers
    _tx_msg.data[5] = 0x22; // 0x22 Starting register: fftorqueCOMM
    // _tx_msg.data[6 to 9] are for the float fftorqueCOMM
    // Query
    _tx_msg.data[10] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
    _tx_msg.data[11] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE
    
    // -----------------------------------------------------------------------------

    // Initial values
    _comm_position = 0.0;
    _comm_velocity = 0.0;
    _comm_fftorque = 0.5;
    _comm_kp_scale = 0.0;
    _comm_kd_scale = 0.0;
    _comm_maxtorqu = 1.0; // 10
}

MoteusPcanMotor::~MoteusPcanMotor(){}

void MoteusPcanMotor::set_commands(float fftorque){
    std::lock_guard<std::mutex> guard(_command_mutex);
    _comm_fftorque = fftorque;
}

void MoteusPcanMotor::get_feedback(float& position, float& velocity, float& torque){
    std::lock_guard<std::mutex> guard(_feedback_mutex);
    position = _position;   
    velocity = _velocity;
    torque = _torque;
}

void MoteusPcanMotor::set_torque_ena(bool torque_ena){ // fonctionne nickel
    std::lock_guard<std::mutex> guard(_torque_ena_mutex);
    if(!torque_ena){ // DISABLE torque
        std::cout << "Motor " << _id << ", torque disabled" << std::endl;
        _torque_ena = false;
    }else{ // ENABLE torque
        std::cout << "Motor " << _id << ", torque enabled" << std::endl;
        if(!_torque_ena){ // Torque is disabled before changing
            std::lock_guard<std::mutex> guard1(_command_mutex);
            std::lock_guard<std::mutex> guard2(_feedback_mutex);
            //_comm_position = _position;
            _comm_position = 0.0;
            _torque_ena = true;
        }
    }
}

bool MoteusPcanMotor::write_read(){ // fonctionne nickel
    bool toque_local;
    {
        std::lock_guard<std::mutex> guard(_torque_ena_mutex);
        toque_local = _torque_ena;
    }
    #ifndef SIMULATE
        // WRITE
        if(!toque_local){
            #ifdef PRINT_TX
                print_message(_msg_tx_stop);
            #endif
            #ifdef USE_PCAN
                _can_device_ptr->Send(_msg_tx_stop);
            #endif
        }else{
            {
                std::lock_guard<std::mutex> guard(_command_mutex);
                memcpy(&_tx_msg.data[6], &_comm_fftorque, sizeof(float));

            }
            #ifdef PRINT_TX
                print_message(_tx_msg);
            #endif
            #ifdef USE_PCAN
                _can_device_ptr->Send(_tx_msg); // send constructed PCAN message, for fftorque only, AND ask for pos, vel, torque
            #else
                std::this_thread::sleep_for(10ms);
            #endif
        }
        // READ
        #ifdef USE_PCAN
            if(!_can_device_ptr->Receive(_rx_msg)){
                return false;
            }
        #else
            std::this_thread::sleep_for(10ms);
        #endif
        #ifdef PRINT_RX
            print_message(msg_rx);
        #endif
        {
            std::lock_guard<std::mutex> guard(_feedback_mutex); 
            memcpy(&_position, &_rx_msg.data[MSGRX_ADDR_POSITION], sizeof(float));
            memcpy(&_velocity, &_rx_msg.data[MSGRX_ADDR_VELOCITY], sizeof(float));
            memcpy(&_torque,   &_rx_msg.data[MSGRX_ADDR_TORQUE],   sizeof(float));
        }
    #else
        std::this_thread::sleep_for(20ms);
        _position = _comm_position;
    #endif

    return true;
}
