#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <igvc/SerialPort.h>
#include <igvc/StringUtils.hpp>

#define DEG_TO_RAD (3.14159265/180.0)

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);

    sleep(1); // sleeping thread for 1 seconds to let IMU set up

    SerialPort port("/dev/ttyUSB0", 57600); // TODO set up udev rules
    port.flush();

    // send a sync request and listen to response - after that the stream should restart
    port.write("#oe0#oat#o1#s12"); // turn on output stream - set output type - request synch token
    string syncResponse = port.readln();
    int syncCounter = 0;
    while (syncResponse != "#SYNCH12") {
        // try to sync again after first time failed - no more than 10 times
        usleep(1000); // sleep for 1 ms
        port.write("#oe0#oat#o1#s12");
        syncResponse = port.readln();
        if (syncCounter++ > 10)
            // give up and terminate node
            throw runtime_error("failed to sync IMU");
    }

    int seq = 0; // sequence of published messgaes - should be monatomicly increasing
    float yawInitial = 0; // initial yaw orientation for publishing relative yaw angle

    while(ros::ok() && port.isOpen()) {
        ros::spinOnce();

        // read the values
        string accel = port.readln();
        string gyro = port.readln();
        string rpy = port.readln();

        // parse the strings read
        vector<string> accelTokens = split(accel, '=');
        if (accelTokens.size() == 2) {
            accel = accelTokens.at(1);
            accelTokens = split(accel, ',');
        } else {
            ROS_ERROR_STREAM("Improperly formatted IMU message - accelerometer data");
        }

        vector<string> gyroTokens = split(gyro, '=');
        if (gyroTokens.size() == 2) {
            gyro = gyroTokens.at(1);
            gyroTokens = split(gyro, ',');
        } else {
            ROS_ERROR_STREAM("Improperly formatted IMU message - gyroscope data");
        }

        vector<string> rpyTokens = split(rpy, '=');
        if (rpyTokens.size() == 2) {
            rpy = rpyTokens.at(1);
            rpyTokens = split(rpy, ',');
        } else {
            ROS_ERROR_STREAM("Improperly formatted IMU message - roll/pitch/yaw data");
        }

        // declare the message to be published
        sensor_msgs::Imu msg;
        msg.header.frame_id = "imu";
        msg.header.stamp = ros::Time::now();
        msg.header.seq = seq++;

        try {
            msg.linear_acceleration.x = stof(accelTokens[0]);
            msg.linear_acceleration.y = -stof(accelTokens[1]); // must be negative - IMU defines y positive as right
            msg.linear_acceleration.z = -stof(accelTokens[2]); // must be negative - IMU defines z positive as down
            msg.linear_acceleration_covariance = {
                    0.04, 1e-6, 1e-6,
                    1e-6, 0.04, 1e-6,
                    1e-6, 1e-6, 0.04
            };

            msg.angular_velocity.x = stof(gyroTokens[0]);
            msg.angular_velocity.y = -stof(gyroTokens[1]);
            msg.angular_velocity.z = -stof(gyroTokens[2]);
            msg.angular_velocity_covariance = {
                    0.02, 1e-6, 1e-6,
                    1e-6, 0.02, 1e-6,
                    1e-6, 1e-6, 0.02
            };

            float yaw = -stof(rpyTokens[0]) * DEG_TO_RAD;
            float pitch = stof(rpyTokens[1]) * DEG_TO_RAD;
            float roll = stof(rpyTokens[2]) * DEG_TO_RAD;
            if (seq == 0)
                yawInitial = yaw; // first time sets initial yaw
            msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yawInitial - yaw);
            msg.orientation_covariance = {
                    0.0025,   1e-6,   1e-6,
                      1e-6, 0.0025,   1e-6,
                      1e-6,   1e-6, 0.0025
            };
        } catch(const invalid_argument &e) {
            ROS_ERROR_STREAM("Exception in parsing IMU message.");
            ROS_ERROR_STREAM(e.what());
        }

        pub.publish(msg);
    }
    return 0;
}
