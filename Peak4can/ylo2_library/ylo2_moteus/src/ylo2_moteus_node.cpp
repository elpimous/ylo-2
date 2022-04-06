#include <iostream>
#include <string.h>
#include <sstream>
#include <list>
#include <iostream>
#include <unistd.h> // used for usleep command

#include "moteus_pcan/moteus_pcan_controller.h"

using namespace std;

// PEAK FDCAN PCI M2 has 4 ports and each port controls one leg (3 moteus_controllers)
MoteusInterfaceMotorsMap interface_motors_map = {
  
  {"/dev/pcanpcifd0", {1,2,3,}},
  {"/dev/pcanpcifd1", {4,5,6,}},
  {"/dev/pcanpcifd2", {7,8,9,}},
  {"/dev/pcanpcifd3", {10,11,12,}},
};

MoteusPcanController controller(interface_motors_map);


// torque switch on/off, and target ID 
void activate_torque_cmd(int motor_id, bool activate)
{
  // send pcan order, using correct port (ex:PCAN_PCIBUS1), target id, and state
  controller._motors[motor_id]->set_torque_ena(activate);
}

// query position, velocity, and torque, for all 12 motors in order (1-12)
void query(int motor_id, float& pos, float& vel, float& tor)
{
  controller._motors[motor_id]->get_feedback(pos, vel, tor); // query values
}

// send fftorque order to specific id, with specific torque
void send_tau(int motor_id, float tor)
{
  controller._motors[motor_id]->set_commands(tor);
}


int main()
{

  float pos, vel, tor;

  // initialize the 4 ports
  if(!controller.is_initialized()){
    std::cerr << "Could not initialize Moteus controllers." << std::endl;
    return 1;
  }

  // start all 12 moteus controllers, and check them
  controller.start();
  if(!controller.all_running())
  {
    std::cerr << "One or more Moteus controllers are not running." << std::endl;
    return 1;
  }

  std::cout << "Motors are running !!!" << std::endl;

  // when activating torque on, we have nearly 1/5 chance to block the loop !!!?!
  activate_torque_cmd(8, true) // torque on
  usleep(5000000);
  
  query(8, pos, vel, tor);// querying values
  std::cout << "Position : " << pos << std::endl;
  std::cout << "velocity : " << vel << std::endl;
  std::cout << "torque : " << tor << std::endl;

  activate_torque_cmd(8, false) // torque off
  usleep(1000);
  
  std::cout << "leaving !" << std::endl; // quit
}
