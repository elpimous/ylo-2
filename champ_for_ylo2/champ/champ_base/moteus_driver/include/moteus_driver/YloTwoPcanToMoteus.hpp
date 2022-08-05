#ifndef PCANTOMOTEUS_HPP
#define PCANTOMOTEUS_HPP

#include <iostream>
#include <ctime>
#include <vector>
#include <math.h>
#include <string.h>
#include <unistd.h> // (for usleep)

#include <PCANBasic.h> // needed lib to communicate with Peak m2canFd board

//define pcan 4 ports to their respective physical adress
#define PCAN_DEV1	PCAN_PCIBUS1
#define PCAN_DEV2	PCAN_PCIBUS2
#define PCAN_DEV3	PCAN_PCIBUS3
#define PCAN_DEV4	PCAN_PCIBUS4

// define needed RX bytes adress
#define MSGRX_ADDR_POSITION 0x02
#define MSGRX_ADDR_VELOCITY 0x06
#define MSGRX_ADDR_TORQUE   0x0A

// a structure for ylo2 controllers setup
struct MotorAdapter{

  public:

    MotorAdapter()
    {
      idx_ = -1;
      sign_ = 1;
      reduction_ = 1.0;
      port_ = 0;
    }

    MotorAdapter(int idx, int sign, double reduction, int port)
    {
      idx_ = idx;
      sign_ = sign;
      reduction_ = reduction;
      port_ = port;
    }

    const int& getIdx() {return idx_;}
    const int& getSign() {return sign_;}
    const double& getReduction() {return reduction_;}
    const int& getPort() {return port_;}

    void setIdx(int idx) {idx_ = idx;}
    void setSign(int sign) {sign_ = sign;}
    void setReduction(double reduction) {reduction_ = reduction;};
    void setPort(int port) {port_ = port;}

  private:

    int idx_;
    int sign_;
    double reduction_;
    int port_;
};

// the YloTwoPcanToMoteus class
class YloTwoPcanToMoteus{

  public:

    // initialize and reset all 4 ports
    bool initialize();

    // send a canFD STOP frame command
    bool send_stop_commands();
    
    // send a canFD frame command
    bool send_pos(int id, int port, float position);

    // query canFD RX Queue, on specific port, for a specific ID
    void get_feedback(int id, int port, float& position, float& velocity, float& torque);

    uint32_t _id; // ID of a moteus controller
    YloTwoPcanToMoteus();
    virtual ~YloTwoPcanToMoteus();

    std::vector<MotorAdapter> motor_adapters_;

  private:

    // for pcanbasic library
    TPCANStatus Status; // the return of a command, to check success
    // Define the compatible Moteus FD Bitrate string
    TPCANBitrateFD BitrateFD = (char*) "f_clock_mhz = 80, nom_brp = 1, nom_tseg1 = 50, nom_tseg2 = 29, nom_sjw = 10, data_brp = 1, data_tseg1 = 8, data_tseg2 = 7, data_sjw = 12";
    TPCANTimestampFD timestamp;
    TPCANMsgFD _stop; // the stop canFD message 
    TPCANMsgFD _tx_msg; // the canFD message to send order to a moteus controller
    TPCANMsgFD _rx_msg; // the canFD message to query values from a moteus controller
    char strMsg[256];


    std::vector<int> pcanPorts_ = {PCAN_DEV1, PCAN_DEV2, PCAN_DEV3, PCAN_DEV4};

    int idx_;
    int sign_;
    double reduction_;
    int port_;

    float _comm_position      = NAN;
    float _comm_velocity      = 1.0;
    float _comm_fftorque      = 8.0; // variable Tau
    float _comm_kp_scale      = 4.0;
    float _comm_kd_scale      = 0.01;
    float _comm_maxtorque     = 8.0; // Max possible torque is NAN value
    float _comm_stop_position = 0.0;
    // float _comm_watchdog_timeout = 0.0;

    float _position; // query variables
    float _velocity;
    float _torque;
};

#endif // PCANTOMOTEUS_HPP