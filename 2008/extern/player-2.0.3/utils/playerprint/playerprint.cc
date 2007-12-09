/*
 * a utility to print out data from any of a number of interfaces
 *
 * $Id: playerprint.cc,v 1.15.2.1 2006/06/09 18:13:51 gerkey Exp $
 *
 */

/** @ingroup utils */
/** @{ */
/** @defgroup util_playerprint playerprint
 * @brief Print sensor data to the console

@par Synopsis

playerprint prints out sensor data to the console.  It is useful mainly
for verifying that a device is working during the setup or debugging
process.  If you want to visualize data, try @ref util_playerv.
If you want to log data to a file, try the @ref driver_writelog
driver.

@par Usage

playerprint is installed alongside player in $prefix/bin, so if player is
in your PATH, then playerprint should also be.  Command-line usage is:
@verbatim
$ playerprint [-r <rate>] [-h <host>] [-p <port>] [-i <index>] <device>
@endverbatim
Where the options are:
- -h &lt;host&gt;: connect to Player on this host (default: localhost)
- -p &lt;port&gt;: connect to Player on this TCP port (default: 6665)
- -r &lt;rate&gt;: request data update at &lt;rate&gt; in Hz (default: 10Hz)
- -i &lt;index&gt;: the index of the device (default: 0)

For example:
<pre>
  $ playerprint -p 7000 laser
</pre>

@par Features

playerprint can print out data for the following kinds of devices:
- @ref interface_actarray
- @ref interface_blobfinder
- @ref interface_bumper
- @ref interface_camera
- @ref interface_dio
- @ref interface_fiducial
- @ref interface_gripper
- @ref interface_ir
- @ref interface_laser
- @ref interface_limb
- @ref interface_localize
- @ref interface_log
- @ref interface_map
- @ref interface_planner
- @ref interface_position1d
- @ref interface_position2d
- @ref interface_position3d
- @ref interface_power
- @ref interface_ptz
- @ref interface_simulation
- @ref interface_sonar
- @ref interface_speech
- @ref interface_truth
- @ref interface_wifi

@author Brian Gerkey

*/

/** @} */

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>  // for atoi(3)
//#include <libplayerc/playerc.h>      // for libplayerc client stuff
#include <libplayerc++/playerc++.h>  // for libplayerc++ client stuff

#include <assert.h>

#define USAGE \
  "USAGE: playerprint [-h <host>] [-p <port>] <device>\n" \
  "       -h <host>:  connect to Player on this host\n" \
  "       -p <port>:  connect to Player on this TCP port\n" \
  "       -r <rate>:  request data update at <rate> in Hz\n" \
  "       -i <index>: the index of the device\n"

std::string g_hostname= PlayerCc::PLAYER_HOSTNAME;
int32_t g_port        = PlayerCc::PLAYER_PORTNUM;
int16_t g_index       = 0;
int16_t g_rate        = 10;

std::string g_device("");

void
print_usage()
{
  using namespace std;
  cout << "USAGE: playerprint [-r <rate>] [-h <host>] [-p <port>] [-i <index>] <device>" << endl;
  cout << "       -h <host>:  connect to Player on this host" << endl;
  cout << "       -p <port>:  connect to Player on this TCP port" << endl;
  cout << "       -r <rate>:  request data update at <rate> in Hz" << endl;
  cout << "       -i <index>: the index of the device" << endl;
}

int
get_options(int argc, char **argv)
{
  int ch=0, errflg=0;
  const char* optflags = "i:h:p:r:";

  while((ch=getopt(argc, argv, optflags))!=-1)
  {
    switch(ch)
    {
      /* case values must match long_options */
      case 'i':
          g_index = atoi(optarg);
          break;
      case 'h':
          g_hostname = optarg;
          break;
      case 'p':
          g_port = atoi(optarg);
          break;
      case 'r':
          std::cerr << "Data rate not yet implemented!" << std::endl;
          g_rate = atoi(optarg);
          break;
      case '?':
      case ':':
      default:
        return (-1);
    }
  }

  if(optind >= argc)
    return(-1);

  g_device = argv[optind];

  return (0);
}

int
main(int argc, char **argv)
{
  using namespace PlayerCc;

  if(get_options(argc, argv) < 0)
  {
    print_usage();
    exit(-1);
  }

  ClientProxy* cp;

  // connect to Player
  PlayerClient client(g_hostname, g_port);

  int code = client.LookupCode(g_device);

  // this code would be much cleaner w/ callbacks on read :)
  switch(code)
  {
//    case PLAYER__CODE:
//      cp = (ClientProxy*)new Proxy(&client,g_index);
//      break;
    case PLAYER_ACTARRAY_CODE:
      cp = (ClientProxy*)new ActArrayProxy(&client,g_index);
      break;
    case PLAYER_BLOBFINDER_CODE:
      cp = (ClientProxy*)new BlobfinderProxy(&client,g_index);
      break;
    case PLAYER_BUMPER_CODE:
      cp = (ClientProxy*)new BumperProxy(&client,g_index);
      break;
    case PLAYER_CAMERA_CODE:
      cp = (ClientProxy*)new CameraProxy(&client,g_index);
      break;
    case PLAYER_DIO_CODE:
      cp = (ClientProxy*)new DioProxy(&client,g_index);
      break;
    case PLAYER_FIDUCIAL_CODE:
      cp = (ClientProxy*)new FiducialProxy(&client,g_index);
      break;
    case PLAYER_GRIPPER_CODE:
      cp = (ClientProxy*)new GripperProxy(&client,g_index);
      break;
    case PLAYER_IR_CODE:
      cp = (ClientProxy*)new IrProxy(&client,g_index);
      break;
    case PLAYER_LASER_CODE:
      cp = (ClientProxy*)new LaserProxy(&client,g_index);
      break;
    case PLAYER_LIMB_CODE:
      cp = (ClientProxy*)new LimbProxy(&client,g_index);
      break;
    case PLAYER_LOCALIZE_CODE:
      cp = (ClientProxy*)new LocalizeProxy(&client,g_index);
      break;
    case PLAYER_LOG_CODE:
      cp = (ClientProxy*)new LogProxy(&client,g_index);
      break;
    case PLAYER_MAP_CODE:
      cp = (ClientProxy*)new MapProxy(&client,g_index);
      break;
    case PLAYER_PLANNER_CODE:
      cp = (ClientProxy*)new PlannerProxy(&client,g_index);
      break;
    case PLAYER_POSITION1D_CODE:
      cp = (ClientProxy*)new Position1dProxy(&client,g_index);
      break;
    case PLAYER_POSITION2D_CODE:
      cp = (ClientProxy*)new Position2dProxy(&client,g_index);
      break;
    case PLAYER_POSITION3D_CODE:
      cp = (ClientProxy*)new Position3dProxy(&client,g_index);
      break;
    case PLAYER_POWER_CODE:
      cp = (ClientProxy*)new PowerProxy(&client,g_index);
      break;
    case PLAYER_PTZ_CODE:
      cp = (ClientProxy*)new PtzProxy(&client,g_index);
      break;
    case PLAYER_SIMULATION_CODE:
      cp = (ClientProxy*)new SimulationProxy(&client,g_index);
      break;
    case PLAYER_SONAR_CODE:
      cp = (ClientProxy*)new SonarProxy(&client,g_index);
      break;
    case PLAYER_SPEECH_CODE:
      cp = (ClientProxy*)new SpeechProxy(&client,g_index);
      break;
//    case PLAYER_TRUTH_CODE:
//      cp = (ClientProxy*)new TruthProxy(&client,g_index);
//      break;
//    case PLAYER_WIFI_CODE:
//      cp = (ClientProxy*)new WifiProxy(&client,g_index);
//      break;
    default:
      std::cout << "Unknown interface " << g_device << std::endl;
      exit(-1);
  }
  assert(cp);

  for(;;)
  {
    /* this blocks until new data comes; 10Hz by default */
    client.Read();

    switch(code)
    {
      case PLAYER_ACTARRAY_CODE:
        std::cout << *reinterpret_cast<ActArrayProxy *> (cp);
        break;
      case PLAYER_BLOBFINDER_CODE:
        std::cout << *reinterpret_cast<BlobfinderProxy *> (cp);
        break;
      case PLAYER_BUMPER_CODE:
        std::cout << *reinterpret_cast<BumperProxy *> (cp);
        break;
      case PLAYER_CAMERA_CODE:
        std::cout << *reinterpret_cast<CameraProxy *> (cp);
        break;
      case PLAYER_DIO_CODE:
        std::cout << *reinterpret_cast<DioProxy *> (cp);
        break;
      case PLAYER_FIDUCIAL_CODE:
        std::cout << *reinterpret_cast<FiducialProxy *> (cp);
        break;
      case PLAYER_GRIPPER_CODE:
        std::cout << *reinterpret_cast<GripperProxy *> (cp);
        break;
      case PLAYER_IR_CODE:
        std::cout << *reinterpret_cast<IrProxy *> (cp);
        break;
      case PLAYER_LASER_CODE:
        std::cout << *reinterpret_cast<LaserProxy *> (cp);
        break;
      case PLAYER_LIMB_CODE:
        std::cout << *reinterpret_cast<LimbProxy *> (cp);
        break;
      case PLAYER_LOCALIZE_CODE:
        std::cout << *reinterpret_cast<LocalizeProxy *> (cp);
        break;
      case PLAYER_LOG_CODE:
        std::cout << *reinterpret_cast<LogProxy *> (cp);
        break;
      case PLAYER_MAP_CODE:
        std::cout << *reinterpret_cast<MapProxy *> (cp);
        break;
      case PLAYER_PLANNER_CODE:
        std::cout << *reinterpret_cast<PlannerProxy *> (cp);
        break;
      case PLAYER_POSITION1D_CODE:
        std::cout << *reinterpret_cast<Position1dProxy *> (cp);
        break;
      case PLAYER_POSITION2D_CODE:
        std::cout << *reinterpret_cast<Position2dProxy *> (cp);
        break;
      case PLAYER_POSITION3D_CODE:
        std::cout << *reinterpret_cast<Position3dProxy *> (cp);
        break;
      case PLAYER_POWER_CODE:
        std::cout << *reinterpret_cast<PowerProxy *> (cp);
        break;
      case PLAYER_PTZ_CODE:
        std::cout << *reinterpret_cast<PtzProxy *> (cp);
        break;
      case PLAYER_SIMULATION_CODE:
        std::cout << *reinterpret_cast<SimulationProxy *> (cp);
        break;
      case PLAYER_SONAR_CODE:
        std::cout << *reinterpret_cast<SonarProxy *> (cp);
        break;
      case PLAYER_SPEECH_CODE:
        std::cout << *reinterpret_cast<SpeechProxy *> (cp);
        break;
//      case PLAYER_TRUTH_CODE:
//        std::cout << *reinterpret_cast<TruthProxy *> (cp);
//        break;
//      case PLAYER_WIFI_CODE:
//        std::cout << *reinterpret_cast<WifiProxy *> (cp);
//        break;
    }

    std::cout << std::endl;
  }
}



