#include <ros/ros.h>
#include <igvc_msgs/action_path.h>
#include <igvc_msgs/velocity_pair.h>
#include <mutex>
#include <chrono>

using namespace igvc_msgs;
using namespace std;
using namespace chrono;

action_pathConstPtr path;
mutex path_mutex;

int path_index = 0;
bool path_reset = false;

void newPath(const action_pathConstPtr &msg)
{
  lock_guard<mutex> lock(mutex);
  path = msg;
  path_index = 0;
  path_reset = true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "path_follower");

  time_point<high_resolution_clock> cmd_start_time;

  ros::NodeHandle n;

  ros::Publisher cmd_pub = n.advertise<velocity_pair>("/motors", 1);

  ros::Rate rate(60);
  while(ros::ok())
  {
    ros::spinOnce();

    { //scope for mutex lock
      lock_guard<mutex> lock(mutex);
      if(path.get() != nullptr)
      {
        if(path_reset)
        {
          cmd_pub.publish(path->actions[path_index]);
          cmd_start_time = high_resolution_clock::now();
        }
        auto elapsed_time = high_resolution_clock::now() -  cmd_start_time;
        if(duration_cast<seconds>(elapsed_time).count() > path->actions[path_index].duration)
        {
          path_index++;
          cmd_pub.publish(path->actions[path_index]);
          cmd_start_time = high_resolution_clock::now();
        }
      }
    }

    rate.sleep();
  }

  return 0;
}
