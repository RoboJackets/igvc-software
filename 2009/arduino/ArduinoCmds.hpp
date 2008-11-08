#ifndef ARDUINOCMDS_HPP_
#define ARDUINOCMDS_HPP_

#define PACKET_ERROR_CMD 0xFF

//1 byte commands, laptop -> arduino
#define ARDUINO_GETSTATUS_CMD 'r'
#define ARDUINO_SETVAR_CMD 'w'
#define ARDUINO_ID_CMD 'i'
#define ARDUINO_RSND_PK_CMD 'p'

//1 byte commands/response arduino -> laptop
#define ARDUINO_ERROR 0xFF
#define ARDUINO_GETSTATUS_RESP 'r'
#define ARDUINO_SETVAR_RESP 'w'
#define ARDUINO_ID_RESP 'i'
#define ARDUINO_RSND_PK_RESP 'p'

// Enums for data settings
enum opttype_t { PUSHPULL = 0, RET_T, INTEROG_DL, SETCLK, RESENDPKT};//interogdl - miliseconds
enum opt_t {PUSH = 0, PULL, SEND_DTICK, SEND_CURRENT};

#endif //ARDUINO_COMMANDS_HPP_
