#include "moteus_pcan/utils.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>

void print_message(const CANDevice::CAN_msg_t& msg){
    std::cout << "CAN Message { " << std::endl;
    std::cout << "  id = 0x" << std::hex << msg.id << std::dec << std::endl;
    std::cout << "  length = " << msg.length << std::endl;
    std::cout << "  data = {" << std::endl;
    for(int i=0; i<msg.length; i++){
        printf("    0x%02X\n", msg.data[i]);
    }
    std::cout << std::dec << std::endl;
    std::cout << "  }" << std::endl;
    std::cout << "}" << std::endl;
}