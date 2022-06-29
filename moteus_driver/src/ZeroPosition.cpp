#include "moteus_driver/YloTwoPcanToMoteus.hpp"

YloTwoPcanToMoteus command;

void send_position(float pos){
  for (unsigned int jj = 0; jj < 12; jj++){
      auto ids  = command.motor_adapters_[jj].getIdx();
      auto sign = command.motor_adapters_[jj].getSign();
      int port  = command.motor_adapters_[jj].getPort();
      command.send_pos(ids, port, pos*sign); 
  }
}

// position: [0.004480198957026005, 1.1018387079238892, -2.0921149253845215, 0.004488484002649784, 1.1042606830596924, -2.0964059829711914, 0.004480198957026005, 1.1018387079238892, -2.0921149253845215, 0.004488484002649784, 1.1042606830596924, -2.0964059829711914]

int main(){

    std::cout << "Start of joints moving to 0.0, in 5 seconds." << std::endl;

    // initialise and reset ports
    command.initialize();
    std::cout << "     -> ports initialized." << std::endl;

    usleep(1000000);
    command.send_stop_commands();
    std::cout << "     -> joints stopped." << std::endl;
    usleep(4000000);

    // moves
    std::cout << "     -> joints moving to 0.0" << std::endl;
    for (int i = 0; i < 1000; i++){
        send_position(0.0);
        usleep(300);
    }
    std::cout << "     -> joints moving to 0.2" << std::endl;
    for (int i = 0; i < 1000; i++){
        send_position(0.05);
        usleep(300);
    }
    std::cout << "     -> joints moving to 0.0" << std::endl;
    for (int i = 0; i < 100000; i++){
        send_position(0.0);
        usleep(300);
    }
    std::cout << "     -> STOPPING !" << std::endl;
    command.send_stop_commands();
}