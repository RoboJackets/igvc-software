#include <igvc/SerialPort.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <igvc/StringUtils.hpp>
#include <vector>

#define DEG_TO_RAD (3.14159265 / 180.0)
#define HALF_TO_FULL_CIRCLE_ANGLE(ang) (((ang) < 0) ? ((ang) + 360) : (ang))

std::list<double> x_accel;
std::list<double> y_accel;
std::list<double> z_accel;

double x_avg;
double y_avg;
double z_avg;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);

  sleep(1);  // sleeping thread for 1 seconds to let IMU set up

  SerialPort port("/dev/igvc_imu", 57600);
  port.flush();

  // send a sync request and listen to response - after that the stream should restart
  port.write("#oe0#oat#o1#s12");  // turn on output stream - set output type - request synch token
  std::string syncResponse = port.readln();

  int syncCounter = 0;
  while (syncResponse != "#SYNCH12" && port.isOpen())
  {
    // try to sync again after first time failed - no more than 10 times
    usleep(1000);  // sleep for 1 ms
    port.write("#oe0#oat#o1#s12");
    syncResponse = port.readln();
    if (syncCounter++ > 10)
      // give up and terminate node
      throw std::runtime_error("failed to sync IMU");
  }

  int seq = 0;  // sequence of published messgaes - should be monatomicly increasing

  while (ros::ok() && port.isOpen())
  {
    ros::spinOnce();

    // read the values
    std::string rpy = port.readln();
    std::string accel = port.readln();
    std::string gyro = port.readln();

    // parse the strings read
    std::vector<std::string> accelTokens = split(accel, '=');
    if (accelTokens.size() == 2)
    {
      accel = accelTokens.at(1);
      accelTokens = split(accel, ',');
    }
    else
    {
      ROS_ERROR_STREAM("Improperly formatted IMU message - accelerometer data");
    }

    std::vector<std::string> gyroTokens = split(gyro, '=');
    if (gyroTokens.size() == 2)
    {
      gyro = gyroTokens.at(1);
      gyroTokens = split(gyro, ',');
    }
    else
    {
      ROS_ERROR_STREAM("Improperly formatted IMU message - gyroscope data");
    }

    std::vector<std::string> rpyTokens = split(rpy, '=');
    if (rpyTokens.size() == 2)
    {
      rpy = rpyTokens.at(1);
      rpyTokens = split(rpy, ',');
    }
    else
    {
      ROS_ERROR_STREAM("Improperly formatted IMU message - roll/pitch/yaw data");
    }

    // declare the message to be published
    sensor_msgs::Imu msg;
    msg.header.frame_id = "imu";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq++;

    try
    {
      double cur_x = stof(accelTokens[0]);
      double cur_y = stof(accelTokens[1]);
      double cur_z = stof(accelTokens[2]);
      msg.linear_acceleration_covariance = { 0.04, 1e-6, 1e-6, 1e-6, 0.04, 1e-6, 1e-6, 1e-6, 0.04 };

      int num = 500;

      x_accel.push_back(cur_x);
      if(x_accel.size() > num) {
        double front = x_accel.front();
        x_accel.pop_front();
        x_avg -= front / num;
      }
      x_avg += cur_x / num;

      z_accel.push_back(cur_z);
      if(z_accel.size() > num) {
        double front = z_accel.front();
        z_accel.pop_front();
        z_avg -= front / num;
      }
      z_avg += cur_z / num;

      y_accel.push_back(cur_y);
      if(y_accel.size() > num) {
        double front = y_accel.front();
        y_accel.pop_front();
        y_avg -= front / num;
      }
      y_avg += cur_y / num;

      msg.linear_acceleration.x = cur_x - x_avg;
      msg.linear_acceleration.y = cur_y - y_avg;
      msg.linear_acceleration.z = cur_z - z_avg;


      msg.angular_velocity.x = stof(gyroTokens[0]);
      msg.angular_velocity.y = stof(gyroTokens[1]);
      msg.angular_velocity.z = stof(gyroTokens[2]);
      msg.angular_velocity_covariance = { 0.02, 1e-6, 1e-6, 1e-6, 0.02, 1e-6, 1e-6, 1e-6, 0.02 };

      float yaw = HALF_TO_FULL_CIRCLE_ANGLE(-(stof(rpyTokens[0]) - 100)) * DEG_TO_RAD;
      float pitch = HALF_TO_FULL_CIRCLE_ANGLE(-stof(rpyTokens[1])) * DEG_TO_RAD;
      float roll = HALF_TO_FULL_CIRCLE_ANGLE(stof(rpyTokens[2]) + 180) * DEG_TO_RAD;

      std::cout << "y raw: " << stof(rpyTokens[0]) - 100 << std::endl;
      std::cout << "y diff: " << stof(rpyTokens[0]) << std::endl;
      std::cout << "y corr: " << stof(rpyTokens[0]) << std::endl;

      msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      msg.orientation_covariance = { 0.0025, 1e-6, 1e-6, 1e-6, 0.0025, 1e-6, 1e-6, 1e-6, 0.0025 };
    }
    catch (const std::invalid_argument& e)
    {
      ROS_ERROR_STREAM("Exception in parsing IMU message.");
      ROS_ERROR_STREAM(e.what());
    }

    pub.publish(msg);
  }
  return 0;
}
