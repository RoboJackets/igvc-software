#ifndef ARDUINOCMDS_HPP_
#define ARDUINOCMDS_HPP_

#define PACKET_ERROR_CMD 0xFF

//1 byte commands, laptop -> arduino
#define ARDUINO_GETSTATUS_CMD 'r'
#define ARDUINO_SETVAR_CMD 'w'
#define ARDUINO_ID_CMD 'i'
#define ARDUINO_RSND_PK_CMD 'p'

//1 byte commands/response arduino -> laptop
#define ARDUINO_ERROR_RESP 0xFF
#define ARDUINO_GETSTATUS_RESP 'r'
#define ARDUINO_SETVAR_RESP 'w'
#define ARDUINO_ID_RESP 'i'
#define ARDUINO_RSND_PK_RESP 'p'


// MC ENUMS
// Enums for data settings
enum mc_opttype_t { MC_PUSHPULL = 0, MC_RET_T, MC_INTEROG_DL, MC_SETCLK, MC_RESENDPKT};//interogdl - miliseconds
enum mc_opt_t {MC_PUSH = 0, MC_PULL, MC_SEND_DTICK, MC_SEND_CURRENT};

//enum for general errors
enum error_t {DROPPED_PACKET, REQUESTED_PACKET_OUT_OF_RANGE};


//enum for sonar options
enum sonar_opttype_t { SN_SETRNG = 0 };


#endif //ARDUINO_COMMANDS_HPP_
