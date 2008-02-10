/* Miscellaneous constants */

/** The largest possible message */
#define PLAYER_MAX_MESSAGE_SIZE 8388608 /*8MB*/
/** Maximum payload in a message */
#define PLAYER_MAX_PAYLOAD_SIZE (PLAYER_MAX_MESSAGE_SIZE - sizeof(player_msghdr_t))
/** Maximum length for a driver name */
#define PLAYER_MAX_DRIVER_STRING_LEN 64
/** The maximum number of devices the server will support. */
#define PLAYER_MAX_DEVICES             256
/** Default maximum length for a message queue */
#define PLAYER_MSGQUEUE_DEFAULT_MAXLEN 32
/** String that is spit back as a banner on connection */
#define PLAYER_IDENT_STRING    "Player v."
/** Length of string that is spit back as a banner on connection */
#define PLAYER_IDENT_STRLEN 32
/** Length of authentication key */
#define PLAYER_KEYLEN       32



/* The Player message types */

/* A data message.  Such messages are asynchronously published from
 * devices, and are usually used to reflect some part of the device's state.
 */
#define PLAYER_MSGTYPE_DATA      1

/* A command message.  Such messages are asynchronously published to
 * devices, and are usually used to change some aspect of the device's state.
 */
#define PLAYER_MSGTYPE_CMD       2

/* A request message.  Such messages are published synchronously to
 * devices, usually to get or set some aspect of the device's state that is
 * not available in data or command messages.  Every request message gets
 * a response message (either PLAYER_MSGTYPE_RESP_ACK or
 * PLAYER_MSGTYPE_RESP_NACK).
 */
#define PLAYER_MSGTYPE_REQ       3

/* A positive response message.  Such messages are published in response
 * to a PLAYER_MSGTYPE_REQ.  This message indicates that the underlying driver
 * received, interpreted, and processed the request.  Any requested data is in
 * the body of this response message.
 */
#define PLAYER_MSGTYPE_RESP_ACK  4

/* A synch message.  Only used in PLAYER_DATAMODE_PULL mode. Sent at the end of 
 * the set of messages that are sent in response to a LAYER_PLAYER_REQ_DATA request.
 */
#define PLAYER_MSGTYPE_SYNCH     5

/* A negative response message.  Such messages are published in response
 * to a PLAYER_MSGTYPE_REQ.  This messages indicates that the underlying
 * driver did not process the message.  Possible causes include: the driver's
 * message queue was full, the driver failed to interpret the request, or the
 * the driver does not support the request.   This message will have no data
 * in the body.*/
#define PLAYER_MSGTYPE_RESP_NACK 6



/* Interface codes - An integer code is assigned to each interface. */

#define PLAYER_NULL_CODE           256 // /dev/null analogue
#define PLAYER_PLAYER_CODE         1   // the server itself
#define PLAYER_POWER_CODE          2   // power subsystem
#define PLAYER_GRIPPER_CODE        3   // gripper
#define PLAYER_POSITION2D_CODE     4   // device that moves about in the plane
#define PLAYER_SONAR_CODE          5   // fixed range-finder
#define PLAYER_LASER_CODE          6   // scanning range-finder
#define PLAYER_BLOBFINDER_CODE     7   // visual blobfinder
#define PLAYER_PTZ_CODE            8   // pan-tilt-zoom unit
#define PLAYER_AUDIO_CODE          9   // audio I/O
#define PLAYER_FIDUCIAL_CODE       10  // fiducial detector
#define PLAYER_SPEECH_CODE         12  // speech I/O
#define PLAYER_GPS_CODE            13  // GPS unit
#define PLAYER_BUMPER_CODE         14  // bumper array
#define PLAYER_TRUTH_CODE          15  // ground-truth (via Stage;
#define PLAYER_DIO_CODE            20  // digital I/O
#define PLAYER_AIO_CODE            21  // analog I/O
#define PLAYER_IR_CODE             22  // IR array
#define PLAYER_WIFI_CODE           23  // wifi card status
#define PLAYER_WAVEFORM_CODE       24  // fetch raw waveforms
#define PLAYER_LOCALIZE_CODE       25  // localization
#define PLAYER_MCOM_CODE           26  // multicoms
#define PLAYER_SOUND_CODE          27  // sound file playback
#define PLAYER_AUDIODSP_CODE       28  // audio dsp I/O
#define PLAYER_AUDIOMIXER_CODE     29  // audio I/O
#define PLAYER_POSITION3D_CODE     30  // 3-D position
#define PLAYER_SIMULATION_CODE     31  // simulators
#define PLAYER_BLINKENLIGHT_CODE   33  // blinking lights
#define PLAYER_NOMAD_CODE          34  // Nomad robot
#define PLAYER_CAMERA_CODE         40  // camera device
#define PLAYER_MAP_CODE            42  // get a map
#define PLAYER_PLANNER_CODE        44  // 2D motion planner
#define PLAYER_LOG_CODE            45  // log read/write control
#define PLAYER_ENERGY_CODE         46  // energy consumption
//#define PLAYER_MOTOR_CODE          47  // motor interface
#define PLAYER_JOYSTICK_CODE       49  // Joytstick
#define PLAYER_SPEECH_RECOGNITION_CODE  50  // speech recognition
#define PLAYER_OPAQUE_CODE         51  // plugin interface
#define PLAYER_POSITION1D_CODE     52  // 1-D position
#define PLAYER_ACTARRAY_CODE       53  // Actuator Array interface
#define PLAYER_LIMB_CODE           54  // Limb interface
#define PLAYER_GRAPHICS2D_CODE     55  // Graphics2D interface
#define PLAYER_RFID_CODE           56  // RFID reader interface
#define PLAYER_WSN_CODE            57  // Wireless Sensor Networks interface
#define PLAYER_GRAPHICS3D_CODE     58  // Graphics3D interface

/* Interface string names
 * Used in configuration file parsing and console output, each interface is
 * assigned a string name.
 */
#define PLAYER_ACTARRAY_STRING        "actarray"
#define PLAYER_AIO_STRING             "aio"
#define PLAYER_AUDIO_STRING           "audio"
#define PLAYER_AUDIODSP_STRING        "audiodsp"
#define PLAYER_AUDIOMIXER_STRING      "audiomixer"
#define PLAYER_BLINKENLIGHT_STRING    "blinkenlight"
#define PLAYER_BLOBFINDER_STRING      "blobfinder"
#define PLAYER_BUMPER_STRING          "bumper"
#define PLAYER_CAMERA_STRING          "camera"
#define PLAYER_ENERGY_STRING          "energy"
#define PLAYER_DIO_STRING             "dio"
#define PLAYER_GRIPPER_STRING         "gripper"
#define PLAYER_FIDUCIAL_STRING        "fiducial"
#define PLAYER_GPS_STRING             "gps"
#define PLAYER_IR_STRING              "ir"
#define PLAYER_JOYSTICK_STRING        "joystick"
#define PLAYER_LASER_STRING           "laser"
#define PLAYER_LIMB_STRING            "limb"
#define PLAYER_LOCALIZE_STRING        "localize"
#define PLAYER_LOG_STRING             "log"
#define PLAYER_MAP_STRING             "map"
#define PLAYER_MCOM_STRING            "mcom"
//#define PLAYER_MOTOR_STRING           "motor"
#define PLAYER_NOMAD_STRING           "nomad"
#define PLAYER_NULL_STRING            "null"
#define PLAYER_OPAQUE_STRING          "opaque"
#define PLAYER_PLANNER_STRING         "planner"
#define PLAYER_PLAYER_STRING          "player"
#define PLAYER_POSITION1D_STRING      "position1d"
#define PLAYER_POSITION2D_STRING      "position2d"
#define PLAYER_POSITION3D_STRING      "position3d"
#define PLAYER_POWER_STRING           "power"
#define PLAYER_PTZ_STRING             "ptz"
#define PLAYER_RFID_STRING            "rfid"
#define PLAYER_SIMULATION_STRING      "simulation"
#define PLAYER_SONAR_STRING           "sonar"
#define PLAYER_SOUND_STRING            "sound"
#define PLAYER_SPEECH_STRING          "speech"
#define PLAYER_SPEECH_RECOGNITION_STRING  "speech_recognition"
#define PLAYER_TRUTH_STRING           "truth"
#define PLAYER_WAVEFORM_STRING        "waveform"
#define PLAYER_WIFI_STRING            "wifi"
#define PLAYER_GRAPHICS2D_STRING       "graphics2d"
#define PLAYER_GRAPHICS3D_STRING       "graphics3d"
#define PLAYER_WSN_STRING             "wsn"



/* Address structures - Device and message address structures. */

/* A device address.
 *	Devices are identified by 12-byte addresses of this form. Some of the
 *	fields are transport-dependent in their interpretation.
 */
typedef struct player_devaddr
{
  /** The "host" on which the device resides.  Transport-dependent. */
  uint32_t host;
  /** The "robot" or device collection in which the device resides.
      Transport-dependent */
  uint32_t robot;
  /** The interface provided by the device; must be one of PLAYER_*_CODE */
  uint16_t interf;
  /** Which device of that interface */
  uint16_t index;
} player_devaddr_t;

/* Generic message header. Every message starts with this header.*/
typedef struct player_msghdr
{
  /** Device to which this message pertains */
  player_devaddr_t addr;
  /** Message type; must be one of PLAYER_MSGTYPE_* */
  uint8_t type;
  /** Message subtype; interface specific */
  uint8_t subtype;
  /** Time associated with message contents (seconds since epoch) */
  double timestamp;
  /** For keeping track of associated messages.  Transport-specific. */
  uint32_t seq;
  /** Size in bytes of the payload to follow */
  uint32_t size;
} player_msghdr_t;
/** @} */


/* General-purpose message structures. These structures often appear inside other structures. */

/* A pose in the plane */
typedef struct player_pose
{
  /** X [m] */
  float px;
  /** Y [m] */
  float py;
  /** yaw [rad] */
  float pa;
} player_pose_t;

/* A rectangular bounding box, used to define the size of an object */
typedef struct player_bbox
{
  /** Width [m] */
  float sw;
  /** Length [m] */
  float sl;
} player_bbox_t;

/* A boolean variable, 0 for false anything else for true */
typedef struct player_bool
{
  /** state */
  uint8_t state;
} player_bool_t;


/* Standard units used in Player messages.
 *	In the interest of using MKS units (http://en.wikipedia.org/wiki/Mks) the
 *	internal message structure will use the following unit base types.

Base units
- kilogram [kg]
- second   [s]
- meter    [m]
- ampere   [A]
- radian   [rad]
- watt     [W]
- degree Celcsius [C]
- hertz    [Hz]
- decibel  [dB]
- volts    [V]

see float.h and limits.h for the limits of floats and integers on your
system

*/


// /////////////////////////////////////////////////////////////////////////////
/*  interface_position2d position2d
 * 	The position2d interface is used to control mobile robot bases in 2D.
 */

/* Request/reply subtype: get geometry */
#define PLAYER_POSITION2D_REQ_GET_GEOM          1
/* Request/reply subtype: motor power */
#define PLAYER_POSITION2D_REQ_MOTOR_POWER       2
/* Request/reply subtype: velocity mode */
#define PLAYER_POSITION2D_REQ_VELOCITY_MODE     3
/* Request/reply subtype: position mode */
#define PLAYER_POSITION2D_REQ_POSITION_MODE     4
/* Request/reply subtype: set odometry */
#define PLAYER_POSITION2D_REQ_SET_ODOM          5
/* Request/reply subtype: reset odometry */
#define PLAYER_POSITION2D_REQ_RESET_ODOM        6
/* Request/reply subtype: set speed PID params */
#define PLAYER_POSITION2D_REQ_SPEED_PID         7
/* Request/reply subtype: set position PID params */
#define PLAYER_POSITION2D_REQ_POSITION_PID      8
/* Request/reply subtype: set speed profile params */
#define PLAYER_POSITION2D_REQ_SPEED_PROF        9

/* Data subtype: state */
#define PLAYER_POSITION2D_DATA_STATE             1
/* Data subtype: geometry */
#define PLAYER_POSITION2D_DATA_GEOM              2

/* Command subtype: velocity command */
#define PLAYER_POSITION2D_CMD_VEL                1
/* Command subtype: position command */
#define PLAYER_POSITION2D_CMD_POS              2
/* Command subtype: carlike command */
#define PLAYER_POSITION2D_CMD_CAR              3

/* Data: state  (PLAYER_POSITION2D_DATA_STATE)
 * The position interface returns data regarding the odometric pose and
 * velocity of the robot, as well as motor stall information.
 */
typedef struct player_position2d_data
{
  /* position [m,m,rad] (x, y, yaw)*/
  player_pose_t pos;
  /* translational velocities [m/s,m/s,rad/s] (x, y, yaw)*/
  player_pose_t vel;
  /* Are the motors stalled? */
  uint8_t stall;
} player_position2d_data_t;

/* Command: velocity ( PLAYER_POSITION2D_CMD_VEL)
 * The position interface accepts new velocities for the robot's motors
 * (drivers may support position control, speed control, or both).
 */
typedef struct player_position2d_cmd_vel
{
  /* translational velocities [m/s,m/s,rad/s] (x, y, yaw)*/
  player_pose_t vel;
  /* Motor state (FALSE is either off or locked, depending on the driver). */
  uint8_t state;
} player_position2d_cmd_vel_t;

/* Command: position ( PLAYER_POSITION2D_CMD_POS)
 * The position interface accepts new positions for the robot's motors
 * (drivers may support position control, speed control, or both).
 */
typedef struct player_position2d_cmd_pos
{
  /** position [m,m,rad] (x, y, yaw)*/
  player_pose_t pos;
  /** velocity at which to move to the position [m/s] or [rad/s] */
  player_pose_t vel;
  /** Motor state (FALSE is either off or locked, depending on the driver). */
  uint8_t state;
} player_position2d_cmd_pos_t;

/* Command: carlike ( PLAYER_POSITION2D_CMD_CAR)
 * The  position interface accepts new carlike velocity commands (speed and
 * turning angle) for the robot's motors (only supported by some drivers).
 */
typedef struct player_position2d_cmd_car
{
  /** forward velocity (m/s) */
  double velocity;
  /** turning angle (rad) */
  double angle;
} player_position2d_cmd_car_t;


/* Data AND Request/reply: geometry.
 * To request robot geometry, send a null PLAYER_POSITION2D_REQ_GET_GEOM
 * request.  Depending on the underlying driver, this message may also be
 * sent as data, with subtype PLAYER_POSITION2D_DATA_GEOM.
 */
typedef struct player_position2d_geom
{
  /* Pose of the robot base, in the robot cs (m, m, rad). */
  player_pose_t pose;
  /* Dimensions of the base (m, m). */
  player_bbox_t size;
} player_position2d_geom_t;

/* Request/reply: Motor power.
 * On some robots, the motor power can be turned on and off from software.
 * To do so, send a  PLAYER_POSITION2D_REQ_MOTOR_POWER request with the
 * format given below, and with the appropriate  state (zero for motors
 * off and non-zero for motors on).  Null response.
 * 
 * Be VERY careful with this command!  You are very likely to start the
 * robot running across the room at high speed with the battery charger
 * still attached.
 */
typedef struct player_position2d_power_config
{
  /* FALSE for off, TRUE for on */
  uint8_t state;
} player_position2d_power_config_t;

/* Request/reply: Change velocity control.
 * Some robots offer different velocity control modes.  It can be changed by
 * sending a  PLAYER_POSITION2D_REQ_VELOCITY_MODE request with the format
 * given below, including the appropriate mode.  No matter which mode is
 * used, the external client interface to the  position device remains
 * the same.  Null response.
 * 
 * The driver_p2os driver offers two modes of velocity control:
 * separate translational and rotational control and direct wheel control.
 * When in the separate mode, the robot's microcontroller internally
 * computes left and right wheel velocities based on the currently commanded
 * translational and rotational velocities and then attenuates these values
 * to match a nice predefined acceleration profile.  When in the direct
 * mode, the microcontroller simply passes on the current left and right
 * wheel velocities.  Essentially, the separate mode offers smoother but
 * slower (lower acceleration) control, and the direct mode offers faster
 * but jerkier (higher acceleration) control.  Player's default is to use
 * the direct mode.  Set @a mode to zero for direct control and non-zero
 * for separate control.

 * For the driver_reb driver, 0 is direct velocity control,
 * 1 is for velocity-based heading PD controller.
 */
typedef struct player_position2d_velocity_mode_config
{
  /* driver-specific */
  uint32_t value;
} player_position2d_velocity_mode_config_t;

/* Request/reply: Reset odometry.
 * To reset the robot's odometry to (x, y, yaw) = (0,0,0), send
 * a PLAYER_POSITION2D_REQ_RESET_ODOM request.  Null response.
 */
typedef struct player_position2d_reset_odom_config
{
} player_position2d_reset_odom_config_t;

/* Request/reply: Change control mode.
 * To change control mode, send a  PLAYER_POSITION2D_REQ_POSITION_MODE request.
 * Null response.
 */
typedef struct player_position2d_position_mode_req
{
  /* 0 for velocity mode, 1 for position mode */
  uint32_t state;
} player_position2d_position_mode_req_t;

/* Request/reply: Set odometry.
 * To set the robot's odometry to a particular state, send a
 * PLAYER_POSITION2D_REQ_SET_ODOM request.  Null response.
 */
typedef struct player_position2d_set_odom_req
{
  /* (x, y, yaw) [m, m, rad] */
  player_pose_t pose;
} player_position2d_set_odom_req_t;

/* Request/reply: Set velocity PID parameters.
 * To set velocity PID parameters, send a  PLAYER_POSITION2D_REQ_SPEED_PID
 * request.  Null response.
 */
typedef struct player_position2d_speed_pid_req
{
  /* PID parameters */
  float kp;
  /* PID parameters */
  float ki;
  /* PID parameters */
  float kd;
} player_position2d_speed_pid_req_t;

/* Request/reply: Set linear speed profile parameters.
 * To set linear speed profile parameters, send a
 *  PLAYER_POSITION2D_REQ_SPEED_PROF request.  Null response.
*/
typedef struct player_position2d_speed_prof_req

  /** max speed [m/s] */
  float speed;
  /* max acceleration [m/s^2] */
  float acc;
} player_position2d_speed_prof_req_t;
