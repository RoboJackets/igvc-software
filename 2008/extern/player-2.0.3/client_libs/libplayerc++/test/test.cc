/***************************************************************************
 * Desc: Test program for the Player C client
 * Author: Andrew Howard
 * Date: 13 May 2002
 # CVS: $Id: test.cc,v 1.5 2006/04/11 15:40:31 veedee Exp $
 **************************************************************************/

#include <unistd.h>

#if HAVE_CONFIG_H
  #include <config.h>
#endif

#include "test.h"

bool use_stage;

int main(int argc, const char *argv[])
{
  const char *host;
  int port;
  int i;
  char *arg;
  const char *device, *sindex; int index;

  // Default host, port
  host = "localhost";
  port = 6665;

  // Read program options
  for (i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], "-h") == 0)
    {
      if(++i >= argc)
      {
        puts("missing hostname");
        exit(-1);
      }
      host = argv[i];
    }
    else if (strcmp(argv[i], "-p") == 0)
    {
      if(++i >= argc)
      {
        puts("missing port");
        exit(-1);
      }
      port = atoi(argv[i]);
    }
    else if(!strcmp(argv[i],"-stage"))
      use_stage = true;
  }

  printf("host [%s:%d]\n", host, port);
  PlayerCc::PlayerClient client(host, port);

  // Run the tests
  for (i = 1; i < argc; i++)
  {
    if (strncmp(argv[i], "--", 2) != 0)
      continue;

    // Get device name and index
    arg = strdup(argv[i]);
    device = strtok(arg + 2, ":");
    sindex = strtok(NULL, "");
    index = (sindex ? atoi(sindex) : 0);

/*
  // Test the ClientProxy and PlayerClient
#ifdef HAVE_BOOST_SIGNALS
  // we need both signals and threads
#ifdef HAVE_BOOST_THREAD
    if(strcmp(device, "client") == 0 || strcmp(device, "all") == 0)
      test_client(&client, index);
#endif
#endif
*/

    // RFID device
    if(strcmp(device, "rfid") == 0 || strcmp(device, "all") == 0)
      test_rfid(&client, index);
    // WSN device
    if(strcmp(device, "wsn") == 0 || strcmp(device, "all") == 0)
      test_wsn(&client, index);
#if 0
    // Power device - a simple one to start with
    if(strcmp(device, "power") == 0 || strcmp(device, "all") == 0)
      test_power(&client, index);

    // DIO device
    if(strcmp(device, "dio") == 0 || strcmp(device, "all") == 0)
      test_dio(&client, index);

    // Position device
    if (strcmp(device, "motor") == 0 || strcmp(device, "all") == 0)
      test_motor(&client, index);
    if (strcmp(device, "position") == 0 || strcmp(device, "all") == 0)
      test_position(&client, index);
    if (strcmp(device, "position2d") == 0 || strcmp(device, "all") == 0)
      test_position2d(&client, index);
    if (strcmp(device, "position3d") == 0 || strcmp(device, "all") == 0)
      test_position3d(&client, index);
    // Position device - position control mode
    // not many robots support this mode but Stage's position model does.
    if (strcmp(device, "position_control") == 0 || strcmp(device, "all") == 0)
      test_position_control(&client, index);

    // Sonar device
    if(strcmp(device, "sonar") == 0 || strcmp(device, "all") == 0)
      test_sonar(&client, index);

    // Name lookup
    if(strcmp(device, "lookup") == 0 || strcmp(device, "all") == 0)
      test_lookup(&client, index);

    // Blobfinder device
    if(strcmp(device, "blobfinder") == 0 || strcmp(device, "all") == 0)
      test_blobfinder(&client, index);

    // Laser device
    if(strcmp(device, "laser") == 0 || strcmp(device, "all") == 0)
      test_laser(&client, index);

    // Fiducial finder device
    if(strcmp(device, "fiducial") == 0 || strcmp(device, "all") == 0)
      test_fiducial(&client, index);

    // PTZ device
    if(strcmp(device, "ptz") == 0 || strcmp(device, "all") == 0)
      test_ptz(&client, index);

    // Speech device
    if(strcmp(device, "speech") == 0 || strcmp(device, "all") == 0)
      test_speech(&client, index);

    // Vision device
    if(strcmp(device, "vision") == 0 || strcmp(device, "all") == 0)
      test_vision(&client, index);


    // GPS device
    if(strcmp(device, "gps") == 0 || strcmp(device, "all") == 0)
      test_gps(&client, index);

    // Gripper device
    if(strcmp(device, "gripper") == 0 || strcmp(device, "all") == 0)
      test_gripper(&client, index);

    // Ground truth device
    if(strcmp(device, "truth") == 0 || strcmp(device, "all") == 0)
      test_truth(&client, index);

    // Laserbeacon device
    if(strcmp(device, "laserbeacon") == 0 || strcmp(device, "all") == 0)
      test_lbd(&client, index);

    // Bumper
    if(strcmp(device, "bumper") == 0 || strcmp(device, "all") == 0)
      test_bumper(&client, index);

    // WiFi
    if(strcmp(device, "wifi") == 0 || strcmp(device, "all") == 0)
      test_wifi(&client, index);

    if(strcmp(device, "log") == 0 || strcmp(device, "all") == 0)
      test_log(&client, index);

    if(strcmp(device, "mcom") == 0 || strcmp(device, "all") == 0)
      test_mcom(&client, index);

    if(strcmp(device, "localize") == 0 || strcmp(device, "all") == 0)
      test_localize(&client, index);

    if(strcmp(device, "audiodsp") == 0 || strcmp(device, "all") == 0)
      test_audiodsp(&client, index);

    if(strcmp(device, "audiomixer") == 0 || strcmp(device, "all") == 0)
      test_audiomixer(&client, index);

    if(strcmp(device, "blinkenlight") == 0 || strcmp(device, "all") == 0)
      test_blinkenlight(&client, index);
#endif

    if(strcmp(device, "camera") == 0 || strcmp(device, "all") == 0)
      test_camera(&client, index);

#if 0
    // BPS device
    /*
    if(strcmp(device, "bps") == 0 || strcmp(device, "all") == 0)
      test_bps(&client, index);
     */

    /*
    // IDAR device
    if(strcmp(device, "idar") == 0 || strcmp(device, "all") == 0)
      test_idar(&client, index);

    // IDAR turret device
    if(strcmp(device, "idarturret") == 0 || strcmp(device, "all") == 0)
      test_idarturret(&client, index);
      */
#endif
    free(arg);
  }

  return 0;
}

