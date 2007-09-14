/**
 * ID for this interface.
 * 
 * Make sure this doesn't conflict with any standard interface IDs
 * defined in <libplayercore/player.h>. These are named according to
 * the form PLAYER_*_CODE.
 */
#define PLAYER_TESTINTERFACE_CODE PLAYER_OPAQUE_CODE //128

/** Data subtype: message */
#define PLAYER_TESTINTERFACE_DATA_MESSAGE 1

/** Request/reply subtype: get message length */
#define PLAYER_TESTINTERFACE_REQ_GET_MESSAGE_LENGTH 1