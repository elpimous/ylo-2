#include "moteus_pcan/moteus_pcan_interface.h"

MoteusPcanInterface::MoteusPcanInterface(const std::string& interface, const std::vector<int>& ids)
    : _initialized(false)
    , _running(false)
    , _fail_count(0)
    , _interface(interface)
    , _freq_counter(0)
{
    // CAN Configuration
    _can_config.bitrate = 1e6; //1mbps
    _can_config.d_bitrate = 2e6; //2mbps
    _can_config.sample_point = .875; //87.5% 
    _can_config.d_sample_point = 0.8; //60%
    _can_config.clock_freq = 80e6; // 80mhz // Read from driver?  
    _can_config.mode_fd = 1; // FD Mode
#ifdef USE_PCAN
    // Open CAN
    if(!_can_device.Open(interface, _can_config, false))
    {
        return;
    }
    _can_device.ClearFilters();
#endif
    // Motors
    for(const auto& id: ids){
        _motors[id] = std::make_shared<MoteusPcanMotor>(id, &_can_device);
    }
    // Everything ok
    _initialized = true;
    return;
}

MoteusPcanInterface::~MoteusPcanInterface(){}

bool MoteusPcanInterface::is_initialized(){
    return _initialized;
}

bool MoteusPcanInterface::is_running(){
    std::lock_guard<std::mutex> guard(_running_mutex);
    return _running;
}

void MoteusPcanInterface::start(){
    {
        std::lock_guard<std::mutex> guard(_running_mutex);
        _running = true;
    }
    if(_initialized){
        _loop_thread = std::make_shared<std::thread>(&MoteusPcanInterface::loop, this);
        _status_loop_thread = std::make_shared<std::thread>(&MoteusPcanInterface::status_loop, this);
    }
}

void MoteusPcanInterface::loop(){
    while(true){
        for(const auto& [id, motor]: _motors){
            if(!motor->write_read()){
                _fail_count++;
            }else{
                _fail_count=0;
            }
            // std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        if(_fail_count>=3){
            std::lock_guard<std::mutex> guard(_running_mutex);
            _running = false;
            std::cerr << "Interface '" << _interface << "' failed, one of its motors is not responding." << std::endl;
            for(const auto& [id, motor]: _motors){
                std::cerr << "  - Motor " << id << std::endl;
            }
            break;
        }
        {
            std::lock_guard<std::mutex> guard(_running_mutex);
            if(!_running){
                break;
            }
        }
        {
            std::lock_guard<std::mutex> guard(_freq_counter_mutex);
            _freq_counter++;
        }
    }
}

void MoteusPcanInterface::status_loop(){
    while(true){
        for(int i=0; i<10; i++){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if(!_running){
                return;
            }
        }
        {
            std::lock_guard<std::mutex> guard(_freq_counter_mutex);
            _freq = _freq_counter;
            _freq_counter = 0;
        }
    }
}

void MoteusPcanInterface::stop(){
    {
        std::lock_guard<std::mutex> guard(_running_mutex);
        _running = false;
    }
    _loop_thread->join();
    _status_loop_thread->join();
}