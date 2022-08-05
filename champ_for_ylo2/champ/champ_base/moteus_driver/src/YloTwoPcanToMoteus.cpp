#include "moteus_driver/YloTwoPcanToMoteus.hpp"

YloTwoPcanToMoteus::YloTwoPcanToMoteus()
{

  pcanPorts_.resize(4); // resize the pcanports_ vector to the number or real ports.

  // NOTE: we should load that from file
  motor_adapters_.resize(12);  // exact motors order, on Ylo2

  //                   IDX                             SIGN                            REDUCTION                          PCAN BOARD PORTS
  // LF
  /*HAA*/ motor_adapters_[0].setIdx(3);  motor_adapters_[0].setSign(-1); motor_adapters_[0].setReduction(6.0); motor_adapters_[0].setPort(PCAN_DEV1);
  /*HFE*/ motor_adapters_[1].setIdx(1);  motor_adapters_[1].setSign(-1); motor_adapters_[1].setReduction(6.0); motor_adapters_[1].setPort(PCAN_DEV1);
  /*KFE*/ motor_adapters_[2].setIdx(2);  motor_adapters_[2].setSign(-1); motor_adapters_[2].setReduction(7.5); motor_adapters_[2].setPort(PCAN_DEV1);

  // LH
  /*HAA*/ motor_adapters_[3].setIdx(9);  motor_adapters_[3].setSign(1);  motor_adapters_[3].setReduction(6.0); motor_adapters_[3].setPort(PCAN_DEV3);
  /*HFE*/ motor_adapters_[4].setIdx(7);  motor_adapters_[4].setSign(-1); motor_adapters_[4].setReduction(6.0); motor_adapters_[4].setPort(PCAN_DEV3);
  /*KFE*/ motor_adapters_[5].setIdx(8);  motor_adapters_[5].setSign(-1); motor_adapters_[5].setReduction(7.5); motor_adapters_[5].setPort(PCAN_DEV3);

  // RF
  /*HAA*/ motor_adapters_[6].setIdx(6);  motor_adapters_[6].setSign(-1); motor_adapters_[6].setReduction(6.0); motor_adapters_[6].setPort(PCAN_DEV2);
  /*HFE*/ motor_adapters_[7].setIdx(4);  motor_adapters_[7].setSign(1);  motor_adapters_[7].setReduction(6.0); motor_adapters_[7].setPort(PCAN_DEV2);
  /*KFE*/ motor_adapters_[8].setIdx(5);  motor_adapters_[8].setSign(1);  motor_adapters_[8].setReduction(7.5); motor_adapters_[8].setPort(PCAN_DEV2);

  // RH
  /*HAA*/ motor_adapters_[9].setIdx(12); motor_adapters_[9].setSign(1);  motor_adapters_[9].setReduction(6.0); motor_adapters_[9].setPort(PCAN_DEV4);
  /*HFE*/ motor_adapters_[10].setIdx(10); motor_adapters_[10].setSign(1); motor_adapters_[10].setReduction(6.0); motor_adapters_[10].setPort(PCAN_DEV4);
  /*KFE*/ motor_adapters_[11].setIdx(11); motor_adapters_[11].setSign(1); motor_adapters_[11].setReduction(7.5); motor_adapters_[11].setPort(PCAN_DEV4);


  /* TX STOP PACKAGE --------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------*/
  _stop.ID = 0x00; // in moteus lib, for example 0x8002 means 80 = query values, and 02 = ID
  _stop.MSGTYPE = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  _stop.DLC = 5; // 5 bytes
  // Write Mode
  _stop.DATA[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
  _stop.DATA[1] = 0x00; // Register to write: MODE
  _stop.DATA[2] = 0x00; // Value to write: STOPPED MODE
  // Query
  _stop.DATA[3] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _stop.DATA[4] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE
  //--------------------------------------------------------------------------------------------

  /* TX POS PACKAGE ----------------------------------------------------------------------------
  ---------------------------------------------------------------------------------------------*/
  _tx_msg.ID = 0x00;
  _tx_msg.MSGTYPE = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  _tx_msg.DLC = 14; // 13 = 32 bytes ; 14 = 48 bytes
  // Write Mode
  _tx_msg.DATA[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
  _tx_msg.DATA[1] = 0x00; // Register to write: MODE
  _tx_msg.DATA[2] = 0x0A; // Value to write: POSITION MODE
  // Write command
  _tx_msg.DATA[3] = 0x0C; // Write floats
  _tx_msg.DATA[4] = 0x07; // Write 7 registers
  _tx_msg.DATA[5] = 0x20; // Starting register: POSITION_COMM
  // Query
  _tx_msg.DATA[34] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _tx_msg.DATA[35] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE

  // moteus controller needs to pad unused bytes to 0x50 !
  _tx_msg.DATA[36] = 0x50;
  _tx_msg.DATA[37] = 0x50;
  _tx_msg.DATA[38] = 0x50;
  _tx_msg.DATA[39] = 0x50;
  _tx_msg.DATA[40] = 0x50;
  _tx_msg.DATA[41] = 0x50;
  _tx_msg.DATA[42] = 0x50;
  _tx_msg.DATA[43] = 0x50;
  _tx_msg.DATA[44] = 0x50;
  _tx_msg.DATA[45] = 0x50;
  _tx_msg.DATA[46] = 0x50;
  _tx_msg.DATA[47] = 0x50;
  //--------------------------------------------------------------------------------------------
}

YloTwoPcanToMoteus::~YloTwoPcanToMoteus()
{
}

bool YloTwoPcanToMoteus::initialize(){

  for (unsigned int pp = 0; pp < 4; pp++){
    // open ports
    Status = CAN_InitializeFD(pcanPorts_[pp], BitrateFD);
    CAN_GetErrorText(Status, 0, strMsg); // check the Status return state
	  if (Status){
		  std::cout << "Error: can't initialize " << pcanPorts_[pp] << " port. Status = " << strMsg << std::endl;
      return(false);
    }
    usleep(10);
    //reset ports
    Status = CAN_Reset(pcanPorts_[pp]);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status){
      std::cout << "Error: can't reset_buffer. " << pcanPorts_[pp] << " port. Status = " << strMsg << std::endl;
      return(false);
    }
    std::cout << "### Initialization & reset : Peak port " << pcanPorts_[pp] << ". " << strMsg << std::endl;
  }
  return(true);
}


bool YloTwoPcanToMoteus::send_stop_commands(){

  for (unsigned int jj = 0; jj < 12; jj++){
    auto idx = motor_adapters_[jj].getIdx();
    auto pport = motor_adapters_[jj].getPort();
    _stop.ID = 0x8000 | idx;
    //std::cout << "jj = " << jj << ", idx = " << idx << ", ID = " << _stop.ID << std::endl;
    Status = CAN_WriteFD(pport, &_stop);
    usleep(10);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status != PCAN_ERROR_OK){
      std::cout << "Error: can't stop motor " << idx << " Status = " << strMsg << std::endl;
      return(false);
    }
  }
  std::cout << "All controllers have stopped with pcan status = " << strMsg << std::endl;
  return(true);
}


bool YloTwoPcanToMoteus::send_pos(int id, int port, float position){

    _comm_stop_position = position;
    _tx_msg.ID = 0x8000 | id;
    memcpy(&_tx_msg.DATA[6],  &_comm_position,      sizeof(float));
    memcpy(&_tx_msg.DATA[10], &_comm_velocity,      sizeof(float));
    memcpy(&_tx_msg.DATA[14], &_comm_fftorque,      sizeof(float));
    memcpy(&_tx_msg.DATA[18], &_comm_kp_scale,      sizeof(float));
    memcpy(&_tx_msg.DATA[22], &_comm_kd_scale,      sizeof(float));
    memcpy(&_tx_msg.DATA[26], &_comm_maxtorque,     sizeof(float));
    memcpy(&_tx_msg.DATA[30], &_comm_stop_position, sizeof(float));
    //std::cout<<("commands to send to moteus : ");
	  //std::copy(std::begin(_tx_msg.DATA), std::end(_tx_msg.DATA), std::ostream_iterator<int>(std::cout, " "));
	  //std::cout << "" << std::endl;
    Status = CAN_WriteFD(port, &_tx_msg);
    usleep(10);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status == PCAN_ERROR_OK){
        return(true);
    }
    std::cout << "error into send_pos() YloTwoPcanToMoteus function : id=" << id << ". Status = " << strMsg << std::endl;
    return(false);
}


void YloTwoPcanToMoteus::get_feedback(int id, int port, float& position, float& velocity, float& torque){
    _rx_msg.ID = 0x8000 | id;
    // read rx queue, until it is empty, to clean it
    //do {
      Status = CAN_ReadFD(port,&_rx_msg, NULL); // read can port
      usleep(10);
      CAN_GetErrorText(Status, 0, strMsg);
      if(Status != PCAN_ERROR_QRCVEMPTY){ // rx queue feeded.
        memcpy(&_position, &_rx_msg.DATA[MSGRX_ADDR_POSITION], sizeof(float));
        memcpy(&_velocity, &_rx_msg.DATA[MSGRX_ADDR_VELOCITY], sizeof(float));
        memcpy(&_torque,   &_rx_msg.DATA[MSGRX_ADDR_TORQUE],   sizeof(float));
        position = _position;   
        velocity = _velocity;
        torque = _torque;
      }      
    //} 
    //while(Status != PCAN_ERROR_QRCVEMPTY);
}
