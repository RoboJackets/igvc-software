#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "SerialInterface.h"

#include <tf/tf.h>
#include <Eigen/Dense>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

// This is the  basic ros-based device driver of IMU
class YostLabDriver : SerialInterface
{
public:
  //! constructor and destructor
  YostLabDriver(ros::NodeHandle& nh_, ros::NodeHandle& priv_nh_);
  ~YostLabDriver();
  //!
  //! \brief run: runs system
  //!
  void run();
  //!
  //! \brief getSoftwareVersion
  //! \return returns software string version
  //!
  std::string getSoftwareVersion();
  //!
  //! \brief restoreFactorySettings resets everything
  //!
  void restoreFactorySettings();
  //!
  //! \brief imu_diagnostic runs diagnostics
  //!
  void imu_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  //!
  //! \brief getAxisDirection
  //! \return returns axis directions
  //!
  std::string getAxisDirection();
  //!
  //! \brief getEulerDecomp
  //! \return decompisition string
  //!
  std::string getEulerDecomp();
  //!
  //! \brief getCalibMode
  //! \return 0 for bias 1 for scale and bias
  //!
  std::string getCalibMode();
  //!
  //! \brief getMIMode
  //! \return 1 if enabled 0 if disabled
  //!
  std::string getMIMode();
  //!
  //! \brief startGyroCalibration
  //!
  void startGyroCalibration();
  //!
  //! \brief setMIMode
  //! \param on True to set , False to Unset
  //!
  void setMIMode(bool on);
  //!
  //! \brief setAndCheckIMUSettings
  //! \param sets some IMU settings and reads some settings from IMU
  //!
  void setAndCheckIMUSettings();
  //!
  //! \brief createAndPublishIMUMessage
  //! \param sets some IMU settings and reads some settings from IMU
  //!
  void createAndPublishIMUMessage(std::vector<double>& parsed_val);
  //!
  //! \param buf the string to parse
  //! \param parsed_vals the previously parsed values
  //! \return number of doubles in the parsed line
  //!
  static int addToParsedVals(const std::string& buf, std::vector<double>& parsed_vals);

private:
  // whether or not to commit the imu settings
  bool commit_settings_;

  // whether gyroscope should be calibrated on startup
  bool calibrate_imu_;

  // IMU orientation correction.
  std::vector<double> imu_orientation_correction_;

  double orientation_rotation_;

  // frame id
  std::string frame_id_;

  // Node Handlers
  ros::NodeHandle yostlab_priv_nh_;
  ros::NodeHandle yostlab_nh_;
  ros::Publisher imu_pub_;
  ros::Publisher magnet_pub_;

  // Diagnostic_updater
  diagnostic_updater::Updater updater;

  std::string software_version_;
  std::string calibration_mode_;
  std::string mi_mode_;
  std::string axis_direction_;
  double sensor_temp_, quaternion_length_, spin_frequency_;
  int msg_counter_;
  ros::Time lastUpdateTime_;
  tf::Quaternion last_quat_;

  // Constants
  const double GRAVITY = 9.80665;
  const double GAUSSTOTESLA = 1e-4;
  static constexpr auto MAX_IMU_TEMP = 185.0;
  static constexpr auto MIN_IMU_TEMP = -40.0;
  static constexpr auto QUATERNION_LENGTH_TOL = 0.02;
  static constexpr auto IMU_TIMEOUT_DELAY = 1.0;

  static constexpr auto SET_GYRO_ENABLED = ":107,1\n";           // enable gyroscope readings as inputs to
                                                                 // the orientation estimation
  static constexpr auto SET_ACCELEROMETER_ENABLED = ":108,1\n";  // enable accelerometer readings as inputs
                                                                 // to the orientation estimation
  static constexpr auto SET_COMPASS_ENABLED = ":109,1\n";        // enable compass readings as inputs to
                                                                 // the orientation estimation
  static constexpr auto SET_AXIS_DIRECTIONS = ":116,001\n";      // X: Right, Y: Forward, Z: Up (right-handed system)
  static constexpr auto SET_CALIB_MODE_SCALE_BIAS = ":169,1\n";  // scale bias
  static constexpr auto SET_REFERENCE_VECTOR_MODE = ":105,1\n";  // single auto continuous
  static constexpr auto SET_FILTER_MODE = ":123,1\n";            // Kalman filter mode
  static constexpr auto SET_RUNNING_AVERAGE_MODE = ":124,1\n";   // places the sensor into a confidence-based running
                                                                 // average mode, which changes the running average
                                                                 // factor based upon the confidence factor
  static constexpr auto SET_RUNNING_AVERAGE_PERCENT =
      ":117,0.45,0.45,0.40,0.45\n";  // sets what percentage of running average to use on a
                                     // component sensor

  static constexpr auto SET_OVERSAMPLE_RATE = "106:100,100,25\n";  // sets the number of times to sample each
                                                                   // component sensor for each iteration of the filter.
  /*
  Slot #1: untared orientation as quaternion [4x float]
  Slot #2: corrected gyroscope vector [3x float]
  Slot #3: corrected acceleration vector [3x float]
  Slot #[4-8]: No Command
  */
  static constexpr auto SET_STREAMING_SLOTS = ":80,6,38,39,40,44,255,255,255\n";

  static constexpr auto BEGIN_GYRO_AUTO_CALIB = ":165\n";  // Performs auto-gyroscope calibration. Sensor should
                                                           // remain still while samples are taken.

  //! Orientation Commands
  static constexpr auto GET_TARED_ORIENTATION_AS_QUATERNION = ":0\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_EULER_ANGLES = ":1\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_ROTATION_MATRIX = ":2\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_AXIS_ANGLE = ":3\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_TWO_VECTOR = ":4\n";
  static constexpr auto GET_DIFFERENCE_QUATERNION = ":5\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_QUATERNION = ":6\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_EULER_ANGLES = ":7\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_ROTATION_MATRIX = ":8\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_AXIS_ANGLE = ":9\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_TWO_VECTOR = ":10\n";
  static constexpr auto GET_TARED_TWO_VECTOR_IN_SENSOR_FRAME = ":11\n";
  static constexpr auto GET_UNTARED_TWO_VECTOR_IN_SENSOR_FRAME = ":12\n";
  //! Corrected Data Commands
  static constexpr auto GET_ALL_CORRECTED_COMPONENT_SENSOR = ":37\n";
  static constexpr auto GET_CORRECTED_GYRO_RATE = ":38\n";
  static constexpr auto GET_CORRECTED_ACCELEROMETER_VECTOR = ":39\n";
  static constexpr auto GET_CORRECTED_COMPASS_VECTOR = ":40\n";
  static constexpr auto GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE = ":41\n";
  static constexpr auto CORRECT_RAW_GYRO_DATA = ":48\n";
  static constexpr auto CORRECT_RAW_ACCEL_DATA = ":49\n";
  static constexpr auto CORRECT_RAW_COMPASS_DATA = ":50\n";
  //! Other Data Commands
  static constexpr auto GET_TEMPERATURE_C = ":43\n";
  static constexpr auto GET_TEMPERATURE_F = ":44\n";
  static constexpr auto GET_CONFIDENCE_FACTOR = ":45\n";
  //! RAW Data Commands
  static constexpr auto GET_ALL_RAW_COMPONENT_SENSOR_DATA = ":64\n";
  static constexpr auto GET_RAW_GYRO_RATE = ":65\n";
  static constexpr auto GET_RAW_ACCEL_DATA = ":66\n";
  static constexpr auto GET_RAW_COMPASS_DATA = ":67\n";
  //! Streaming Commands
  static constexpr auto SET_STREAMING_SLOTS_EULER_TEMP = ":80,1,43,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_EULER_QUATERNION = ":80,1,0,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION_EULER = ":80,0,1,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_EULER = ":80,1,255,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION = ":80,0,255,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION_CORRECTED_GYRO_ACCELERATION_LINEAR_IN_GLOBAL = ":80,0,38,41,255,"
                                                                                                      "255,255,255,"
                                                                                                      "255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION_CORRECTED_GYRO_ACCELERATION_LINEAR = ":80,0,38,39,255,255,255,"
                                                                                            "255,255\n";
  static constexpr auto GET_STREAMING_SLOTS = ":81\n";
  static constexpr auto SET_STREAMING_TIMING_5_MS = ":82,5000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_10_MS = ":82,10000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_100_MS = ":82,100000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_1000_MS = ":82,1000000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_5000_MS = ":82,5000000,0,0\n";
  static constexpr auto GET_STREAMING_TIMING = ":83\n";
  static constexpr auto GET_STREAMING_BATCH = ":84\n";
  static constexpr auto START_STREAMING = ":85\n";
  static constexpr auto STOP_STREAMING = ":86\n";
  static constexpr auto UPDATE_CURRENT_TIMESTAMP = ":95\n";
  //! Configuration Read Commands
  static constexpr auto GET_AXIS_DIRECTION = ":143\n";
  static constexpr auto GET_FILTER_MODE = ":152\n";
  static constexpr auto GET_EULER_DECOMPOSTION_ORDER = ":156\n";
  static constexpr auto GET_MI_MODE_ENABLED = ":136\n";
  //! Configuration Write Commands
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_XYZ = ":16,0\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_YZX = ":16,1\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_ZXY = ":16,2\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_ZYX = ":16,3\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_XZY = ":16,4\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_YXZ = ":16,5\n";
  static constexpr auto OFFSET_WITH_CURRENT_ORIENTATION = ":19\n";
  static constexpr auto TARE_WITH_CURRENT_ORIENTATION = ":96\n";
  static constexpr auto TARE_WITH_CURRENT_QUATERNION = ":97\n";
  static constexpr auto SET_MI_MODE_ENABLED = ":112,1\n";
  static constexpr auto SET_MI_MODE_DISABLED = ":112,0\n";
  static constexpr auto SET_AXIS_DIRECTIONS_ENU = ":116,8\n";
  static constexpr auto SET_AXIS_DIRECTIONS_DEFAULT = ":116,000\n";
  static constexpr auto SET_GYRO_RANGE = ":125,1\n";
  static constexpr auto COMMIT_SETTINGS = ":225\n";
  static constexpr auto SET_FILTER_MODE_IMU = ":123,1\n";
  //! Calibration Commands
  static constexpr auto SET_CALIB_MODE_BIAS = ":169,0\n";
  static constexpr auto GET_CALIB_MODE = ":170\n";
  static constexpr auto BEGIN_MI_MODE_FIELD_CALIBRATION = ":114\n";  // Begins the calibration process for MI mode. The
                                                                     // sensor should be left in a magnetically
                                                                     // unperturbed area for 3-4 seconds after this is
                                                                     // called for calibration to succeed.
  //! System Commands
  static constexpr auto GET_FIRMWARE_VERSION_STRING = ":223\n";
  static constexpr auto RESTORE_FACTORY_SETTINGS = ":224\n";
  static constexpr auto SOFTWARE_RESET = ":226\n";
  //! logger space
  static constexpr auto logger = "[ YostImuDriver ] ";
};  // YostLabDriver
