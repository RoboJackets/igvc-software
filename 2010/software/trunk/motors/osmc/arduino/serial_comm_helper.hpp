#ifndef SERIAL_COMM_HELPER_HPP_
#define SERIAL_COMM_HELPER_HPP_

#define TIMEOUT_LENGTH_MILLIS 1000

bool serialReadBytesTimeout(byte len, byte * msg);

void serialPrintBytes(void *data, size_t numBytes);

#endif
