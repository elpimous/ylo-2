#include "moteus_pcan/moteus_pcan_motor.h"
#include "moteus_pcan/utils.h"

#define MSGTX_ADDR_POSITION 0x06
#define MSGTX_ADDR_VELOCITY 0x0A
#define MSGTX_ADDR_FFTORQUE 0x0E
#define MSGTX_ADDR_KP_SCALE 0x12
#define MSGTX_ADDR_KD_SCALE 0x16
#define MSGTX_ADDR_MAXTORQU 0x1A

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
    _msg_tx_stop.id = 0x8000 | id;
    _msg_tx_stop.length = 5;
    // Write Mode
    _msg_tx_stop.data[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
    _msg_tx_stop.data[1] = 0x00; // Register to write: MODE
    _msg_tx_stop.data[2] = 0x00; // Value to write: STOPPED MODE
    // Query
    _msg_tx_stop.data[3] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
    _msg_tx_stop.data[4] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE



    // TX POS PACKAGE
    _msg_tx_pos.id = 0x8000 | id;
    _msg_tx_pos.length = 32;
    // Write Mode
    _msg_tx_pos.data[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
    _msg_tx_pos.data[1] = 0x00; // Register to write: MODE
    _msg_tx_pos.data[2] = 0x0A; // Value to write: POSITION MODE
    // Write command
    _msg_tx_pos.data[3] = 0x0C; // Write floats
    _msg_tx_pos.data[4] = 0x06; // Write 6 registers
    _msg_tx_pos.data[5] = 0x20; // Starting register: positionCOMM, velocityCOMM, fftorqueCOMM, KP_SCALE, KD_SCALE, MAX_TORQUE
    // Query
    _msg_tx_pos.data[30] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
    _msg_tx_pos.data[31] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE
    
    // Initial values
    _comm_position = 0.0;
    _comm_velocity = 0.0;
    _comm_fftorque = 0.0;
    _comm_kp_scale = 60.0;
    _comm_kd_scale = 5.0;
    _comm_maxtorqu = 5.0;
}

MoteusPcanMotor::~MoteusPcanMotor(){}

void MoteusPcanMotor::set_commands(float position, float velocity, float fftorque, float kp_scale, float kd_scale){
    std::lock_guard<std::mutex> guard(_command_mutex);
    _comm_position = position;
    _comm_velocity = velocity;
    _comm_fftorque = fftorque;
    _comm_kp_scale = kp_scale;
    _comm_kd_scale = kd_scale;
}

void MoteusPcanMotor::set_commands(float position, float velocity, float fftorque){
    std::lock_guard<std::mutex> guard(_command_mutex);
    _comm_position = position;
    _comm_velocity = velocity;
    _comm_fftorque = fftorque;
}

void MoteusPcanMotor::set_commands(float position, float velocity){
    std::lock_guard<std::mutex> guard(_command_mutex);
    _comm_position = position;
    _comm_velocity = velocity;
}

void MoteusPcanMotor::set_commands(float position){
    std::lock_guard<std::mutex> guard(_command_mutex);
    _comm_position = position;
}

void MoteusPcanMotor::get_feedback(float& position, float& velocity, float& torque){
    std::lock_guard<std::mutex> guard(_feedback_mutex);
    position = _position;   
    velocity = _velocity;
    torque = _torque;
}

void MoteusPcanMotor::set_torque_ena(bool torque_ena){
    std::lock_guard<std::mutex> guard(_torque_ena_mutex);
    if(!torque_ena){ // DISABLE torque
        std::cout << "Motor " << _id << ", torque disabled" << std::endl;
        _torque_ena = false;
    }else{ // ENABLE torque
        std::cout << "Motor " << _id << ", torque enabled" << std::endl;
        if(!_torque_ena){ // Torque is disabled before changing
            std::lock_guard<std::mutex> guard1(_command_mutex);
            std::lock_guard<std::mutex> guard2(_feedback_mutex);
            _comm_position = _position;
            _torque_ena = true;
        }
    }
}

bool MoteusPcanMotor::write_read(){
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
                memcpy(&_msg_tx_pos.data[MSGTX_ADDR_POSITION], &_comm_position, sizeof(float));
                memcpy(&_msg_tx_pos.data[MSGTX_ADDR_VELOCITY], &_comm_velocity, sizeof(float));
                memcpy(&_msg_tx_pos.data[MSGTX_ADDR_FFTORQUE], &_comm_fftorque, sizeof(float));
                memcpy(&_msg_tx_pos.data[MSGTX_ADDR_KP_SCALE], &_comm_kp_scale, sizeof(float));
                memcpy(&_msg_tx_pos.data[MSGTX_ADDR_KD_SCALE], &_comm_kd_scale, sizeof(float));
                memcpy(&_msg_tx_pos.data[MSGTX_ADDR_MAXTORQU], &_comm_maxtorqu, sizeof(float));
            }
            #ifdef PRINT_TX
                print_message(_msg_tx_pos);
            #endif
            #ifdef USE_PCAN
                _can_device_ptr->Send(_msg_tx_pos);
            #else
                std::this_thread::sleep_for(10ms);
            #endif
        }
        // READ
        #ifdef USE_PCAN
            if(!_can_device_ptr->Receive(_msg_rx)){
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
            memcpy(&_position, &_msg_rx.data[MSGRX_ADDR_POSITION], sizeof(float));
            memcpy(&_velocity, &_msg_rx.data[MSGRX_ADDR_VELOCITY], sizeof(float));
            memcpy(&_torque,   &_msg_rx.data[MSGRX_ADDR_TORQUE],   sizeof(float));
        }
    #else
        std::this_thread::sleep_for(20ms);
        _position = _comm_position;
    #endif
    return true;
}
