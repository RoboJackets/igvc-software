#ifndef SRC_MOTION_PROFILER_H
#define SRC_MOTION_PROFILER_H

#include <igvc_msgs/trajectory.h>

namespace motion_profiler {
  void profileTrajectory(igvc_msgs::trajectoryPtr trajectory_ptr);
}

#endif //SRC_MOTION_PROFILER_H
