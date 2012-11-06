#ifndef SERIAL_COMM_HELPER_HPP_
#define SERIAL_COMM_HELPER_HPP_

#include "common_defines.hpp"

bool serialReadBytesTimeout(byte len, byte * msg);

void serialPrintBytes(void *data, size_t numBytes);

#endif
