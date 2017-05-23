#include <igvc_msgs/action_path.h>
#include <igvc_msgs/lights.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <chrono>
#include <mutex>

igvc_msgs::action_pathConstPtr path;
igvc_msgs::velocity_pairConstPtr velocity;
std::mutex path_mutex;

size_t path_index = 0;
bool path_reset = false;

void newPath_callback(const igvc_msgs::action_pathConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(std::mutex);
  ROS_INFO("Follower got path");
  path = msg;
  path_index = 0;
  path_reset = true;
}

void encoders_callback(const igvc_msgs::velocity_pairConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(std::mutex);
  velocity = msg;
}

igvc_msgs::velocity_pair get_velocity(int time, igvc_msgs::velocity_pair desired_velocity)
{
  igvc_msgs::velocity_pair result;
  //std::cout << "current left = " << velocity->left_velocity << " right = " << velocity->right_velocity << std::endl;
  //std::cout << "desired left = " << desired_velocity.left_velocity << " right = " << desired_velocity.right_velocity << " duration = " << desired_velocity.duration << std::endl;
  double left_velocity_m = (desired_velocity.left_velocity - velocity->left_velocity) / (desired_velocity.duration - time);
  double right_velocity_m = (desired_velocity.right_velocity - velocity->right_velocity) / (desired_velocity.duration - time);
  double step = desired_velocity.duration / 2.0;
  double left_velocity_b = desired_velocity.left_velocity - left_velocity_m * desired_velocity.duration;
  double right_velocity_b = desired_velocity.right_velocity - right_velocity_m * desired_velocity.duration;
  result.right_velocity = right_velocity_m * (time + step) + right_velocity_b;
  result.left_velocity = left_velocity_m * (time + step) + left_velocity_b;
  //std::cout << " result left = " << result.left_velocity << " right = " << result.right_velocity << std::endl;
  result.duration = step;
  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower");

  std::chrono::time_point<std::chrono::high_resolution_clock> cmd_start_time;

  ros::NodeHandle n;

  ros::Publisher cmd_pub = n.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  ros::Publisher lights_pub = n.advertise<igvc_msgs::lights>("/lights", 1);

  ros::Subscriber path_sub = n.subscribe("/path", 1, newPath_callback);

  ros::Subscriber encoder_sub = n.subscribe("/encoders", 1, encoders_callback);

  igvc_msgs::lights lights_cmd;
  lights_cmd.safety_flashing = true;

  ros::Rate rate(120);
  while (ros::ok())
  {
    lights_pub.publish(lights_cmd);
    ros::spinOnce();

    {  // scope for mutex lock
      std::lock_guard<std::mutex> lock(std::mutex);

      if (path.get() != nullptr)
      {
        auto elapsed_time = std::chrono::high_resolution_clock::now() - cmd_start_time;
        if (path->actions.empty())
        {
          ROS_INFO("Path empty.");
          igvc_msgs::velocity_pair vel;
          vel.left_velocity = 0.;
          vel.right_velocity = 0.;
          vel.duration = 0.1;
          cmd_pub.publish(get_velocity(0, vel));
          path.reset();
        }
        else if (path_reset)
        {
          cmd_start_time = std::chrono::high_resolution_clock::now();
          cmd_pub.publish(get_velocity(0, path->actions[path_index]));
          path_reset = false;
        }
        else if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() >
                 path->actions[path_index].duration)
        {
          if (path_index < path->actions.size() - 1)
          {
            path_index++;
            cmd_pub.publish(get_velocity(std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count(), path->actions[path_index]));
            ROS_INFO("Command Published.");
            cmd_start_time = std::chrono::high_resolution_clock::now();
          }
          else
          {
            igvc_msgs::velocity_pair vel;
            vel.left_velocity = 0.;
            vel.right_velocity = 0.;
            cmd_pub.publish(vel);
            path.reset();
          }
        }
      }
    }

    rate.sleep();
  }

  return 0;
}
