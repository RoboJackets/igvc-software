#ifndef SPI_H
#define SPI_H

// TODO: document the side-effects

/* Method of SPI*/
//#define BITBANG_SPI 1

/**
 *
 */
void InitSPI(void);

/**
 * Get the specified number of bytes. Not multi-thread safe.
 */
bool SPIReadBytes(void *data, int numBytes, int inputPin, int slaveSelectPin, int clockPin)

/**
 * Send the specified number of bytes. Not multi-thread safe.
 */
bool SPISendBytes(void *data, int numBytes, int inputPin, int slaveSelectPin, int clockPin);

/**
 * Send and recieve the specified number of bytes via hardware interface.
 */
bool SPITransfer(void *outdata, void *indata, unsigned int numBytes){

#endif /* SPI_H */

