#ifndef UTILS_H
#define UTILS_H

#include "PCANDevice.h"

void print_message(const CANDevice::CAN_msg_t& msg);

#define ELEMENT_IN_VECTOR(_element, _vector) \
    (std::find(_vector.begin(), _vector.end(), _element) != _vector.end())

#define KEY_IN_MAP(_key, _map) _map.count(_key)

#endif