/*
 * grip.cc
 *
 * a simple demo to open and close the gripper
 */

#include <libplayerc++/playerc++.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#include "args.h"

int main(int argc, char **argv)
{
  parse_args(argc,argv);

  using namespace PlayerCc;

  /* Connect to Player server */
  PlayerClient robot(gHostname, gPort);

  /* Request sensor data */
  GripperProxy gp(&robot, gIndex);

  int count = 0;
  bool gripopen = true;
  for(;;)
  {

    robot.Read();

    std::cout << gp << std::endl;

    if(!(++count % 100))
    {
      if(gripopen)
      {
        std::cout << "OPEN" << std::endl;
        gp.SetGrip(GRIPopen,0);
      }
      else
      {
        std::cout << "CLOSE" << std::endl;
        gp.SetGrip(GRIPclose,0);
      }

      gripopen=!gripopen;
      //count=0;
    }
  }

  return(0);
}

