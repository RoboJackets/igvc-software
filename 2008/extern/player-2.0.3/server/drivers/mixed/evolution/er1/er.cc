/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000-2003
 *
 *		David Feil-Seifer
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 *
 * Driver for the "ER" robots, made by Evolution Robotics.    
 *
 * Some of this code is borrowed and/or adapted from the player
 * module of trogdor; thanks to the author of that module.
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_er1 er1
 * @brief Evolution ER1 mobile robot

@todo This driver is currently disabled because it needs to be updated to
the Player 2.0 API.

The er1 driver provides position control of the Evolution Robotics'
ER1 and ERSDK robots.

This driver is new and not thoroughly tested.  The odometry cannot be
trusted to give accurate readings.

You will need a kernel driver to allow the serial port to be seen.
This driver, and news about the player driver can be found <a
href="http://www-robotics.usc.edu/~dfseifer/project-erplayer.php">here</a>.

@todo Implement IR and power interfaces.

NOT DOING: I don't have a gripper, if someone has code for a gripper,
by all means contribute it.  It would be welcome to the mix.

@par Compile-time dependencies

- &lt;asm/ioctls.h&gt;

@par Provides

- @ref interface_position2d

@par Requires

- none

@par Supported configuration requests

- PLAYER_POSITION_GET_GEOM_REQ
- PLAYER_POSITION_MOTOR_POWER_REQ

@par Configuration file options

- port (string)
  - Default: "/dev/usb/ttyUSB0"
  - Serial port used to communicate with the robot.
- axle (length)
  - Default: 0.38 m
  - The distance between the motorized wheels
- motor_dir (integer)
  - Default: 1
  - Direction of the motors; should be 1 or -1.  If the left motor is
    plugged in to the motor 1 port on the RCM, put -1 here instead
- debug (integer)
  - Default: 0
  - Should the driver print out debug messages?
  
@par Example 

@verbatim
driver
(
  name "er1"
  provides ["position2d:0"]
)
@endverbatim

@author David Feil-Seifer
*/
/** @} */


#if HAVE_CONFIG_H
  #include <config.h>
#endif

#include "er.h"
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <netinet/in.h>  /* for struct sockaddr_in, htons(3) */
#include <math.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <libplayercore/playercore.h>

static void StopRobot(void* erdev);


// initialization function
Driver* ER_Init( ConfigFile* cf, int section)
{
	return((Driver*)(new ER(cf, section)));
}


// a driver registration function
void 
ER_Register(DriverTable* table)
{
  table->AddDriver("er1", ER_Init);
}

ER::ER(ConfigFile* cf, int section) :
  Driver(cf,section)
{


  // zero ids, so that we'll know later which interfaces were requested
  memset(&this->position_id, 0, sizeof(position_id));
  this->position_subscriptions = 0;
  
  printf( "discovering drivers\n" );
  
  // Do we create a position interface?
  if(cf->ReadDeviceAddr(&(this->position_id), section, "provides", 
                      PLAYER_POSITION2D_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->position_id) != 0)
    {
      this->SetError(-1);    
      return;
    }
  }

  //::initialize_robot_params();
  
  _fd = -1;
  _tc_num = new int[3];
  _tc_num[0] = 2;
  _tc_num[1] = 0;
  _tc_num[2] = 185;
  this->_serial_port = cf->ReadString(section, "port", ER_DEFAULT_PORT);
	_need_to_set_speed = true;
	
  //read in axle radius
	_axle_length = cf->ReadFloat( section, "axle", ER_DEFAULT_AXLE_LENGTH );

  //read in left motor and right motor direction
  int dir = cf->ReadInt(section,"motor_dir", 1);
	_motor_0_dir = dir * ER_DEFAULT_MOTOR_0_DIR;
	_motor_1_dir = dir * ER_DEFAULT_MOTOR_1_DIR;

	_debug = 1== cf->ReadInt( section, "debug", 0 );

}
/////////////////////
// bus fcns 
/////////////////////

int
ER::ReadBuf(unsigned char* s, size_t len)
{

	int thisnumread;
	size_t numread = 0;

  while(numread < len)
  {
//  	printf( "len=%d numread=%d\n", len, numread );
    if((thisnumread = read(this->_fd,s+numread,len-numread)) < 0)
    {
      printf("read() failed: %s\n", strerror(errno));
      return(-1);
    }
    if(thisnumread == 0)
      printf("short read\n");
    numread += thisnumread;
  }
  
//  printf("read:\n");
//  for(size_t i=0;i<numread;i++) printf("0x%02x\n", s[i]);
  
  return(0);
}

int
ER::WriteBuf(unsigned char* s, size_t len)
{

  size_t numwritten;
  int thisnumwritten;
  numwritten=0;
  while(numwritten < len)
  {
    if((thisnumwritten = write(this->_fd,s+numwritten,len-numwritten)) < 0)
    {
      if(!this->_fd_blocking && errno == EAGAIN)
      {
        usleep(ER_DELAY_US);
        continue;
      }
      printf("write() failed: %s\n", strerror(errno));
      return(-1);
    }
    numwritten += thisnumwritten;
  }
 
	ioctl( this->_fd, TIOCMSET, _tc_num );
	if( _tc_num[0] == 2 ) { _tc_num[0] = 0; }
	else { _tc_num[0] = 2; }


  return 0;
}

int
ER::send_command( unsigned char address, unsigned char c, int ret_num, unsigned char * ret )
{
  
  unsigned char cmd[4];
  
  cmd[0] = address;
  cmd[2] = 0x00;
  cmd[3] = c;
  
  //compute checksum
  int chk = 0x100;
  chk -= cmd[0];
  chk -= cmd[2];
  chk -= cmd[3];
  
  cmd[1] = (unsigned char) chk;
  
  int result = WriteBuf( cmd, 4 );
  
  if( result < 0 )
  {
    printf( "failed to send command\n" );
  }
  
  if( ret > 0 )
  {
    usleep( ER_DELAY_US );
  
    if( ReadBuf( ret, ret_num ) < 0 )
    {
      printf( "failed to read response\n" );
    }
  }


//	PLAYER_WARN1( "cmd: 0x%4x", *((int *) cmd) );

  return result;
}



int
ER::send_command_2_arg( unsigned char address, unsigned char c, int arg, int ret_num, unsigned char * ret )
{
  
  unsigned char cmd[6];
  
  cmd[0] = address;
  cmd[2] = 0x00;
  cmd[3] = c;
  
  int a = htons( arg );
  
  cmd[5] = (a >> 8) & 0xFF;
  cmd[4] = (a >> 0) & 0xFF;
  
  //compute checksum
  int chk = 0x100;
  chk -= cmd[0];
  chk -= cmd[2];
  chk -= cmd[3];
  chk -= cmd[4];
  chk -= cmd[5];
  
  cmd[1] = (unsigned char) chk;
  
  int result = WriteBuf( cmd, 6 );
  
  if( result < 0 )
  {
    printf( "failed to send command\n" );
  }
  
  
  if( ret > 0 )
  {
    usleep( ER_DELAY_US );
    if( ReadBuf( ret, ret_num ) < 0 )
    {
      printf( "failed to read response\n" );
    }
  }
//	PLAYER_WARN1( "cmd: 0x%4x", *((int *) cmd) );
//	PLAYER_WARN1( "cmd: 0x%4x", *((int *) &(cmd[4])) );
  return result;
}

int
ER::send_command_4_arg( unsigned char address, unsigned char c, int arg, int ret_num, unsigned char * ret )
{
  
  unsigned char cmd[8];
  
  cmd[0] = address;
  cmd[2] = 0x00;
  cmd[3] = c;
  
  int a = htonl( arg );
  cmd[7] = (a >> 24) & 0xFF;
  cmd[6] = (a >> 16) & 0xFF;
  cmd[5] = (a >> 8 ) & 0xFF;
  cmd[4] = (a >> 0 ) & 0xFF;
  
  //compute checksum
  int chk = 0x100;
  chk -= cmd[0];
  chk -= cmd[2];
  chk -= cmd[3];
  chk -= cmd[4];
  chk -= cmd[5];
  chk -= cmd[6];
  chk -= cmd[7];
  
  cmd[1] = (unsigned char) chk;
  
  int result = WriteBuf( cmd, 8 );
  
  if( result < 0 )
  {
    printf( "failed to send command\n" );
  }
  
  if( ret > 0 )
  {
    usleep( ER_DELAY_US );
    if( ReadBuf( ret, ret_num ) < 0 )
    {
      printf( "failed to read response\n" );
    }
  }
//	PLAYER_WARN1( "cmd: 0x%4x", *((int *) cmd) );
//	PLAYER_WARN1( "cmd: 0x%4x", *((int *) &(cmd[4])) );
  
  return result;
}

//////////////////////////////
// robot initializations
//////////////////////////////

int
ER::InitRobot()
{

  // initialize the robot
  unsigned char buf[6];
  usleep(ER_DELAY_US);
  if(send_command( MOTOR_0, GETVERSION, 6, buf ) < 0)
  {
    printf("failed to initialize robot\n");
    return -1;
  }

  /* TODO check return value to match 0x00A934100013 */
  if(send_command( MOTOR_1, GETVERSION, 6, buf ) < 0)
  {
    printf("failed to initialize robot\n");
    return -1;
  }
  /* TODO check return value to match 0x00A934100013 */

  _tc_num[2] = 25;
  _stopped = true;
  return(0);
}

int
ER::InitOdom()
{

  unsigned char ret[8];
  
  //try leaving the getVersion out
  send_command( MOTOR_0, RESET, 2, ret );
  send_command_2_arg( MOTOR_0, SETMOTORCMD, 0, 2, ret );
  send_command_2_arg( MOTOR_0, SETLIMITSWITCHMODE, 0, 2, ret );
  send_command_2_arg( MOTOR_0, SETPROFILEMODE, 0x0001, 2, ret );
  send_command_4_arg( MOTOR_0, SETVEL, 0, 2, ret );  
  send_command_4_arg( MOTOR_0, SETACCEL, 0, 2, ret );  
  send_command_4_arg( MOTOR_0, SETDECEL, 0, 2, ret );  
  
  //same for motor 1
  send_command( MOTOR_1, RESET, 2, ret );
  send_command_2_arg( MOTOR_1, SETMOTORCMD, 0, 2, ret );
  send_command_2_arg( MOTOR_1, SETLIMITSWITCHMODE, 0, 2, ret );
  send_command_2_arg( MOTOR_1, SETPROFILEMODE, 0x0001, 2, ret );
  send_command_4_arg( MOTOR_1, SETVEL, 0, 2, ret );  
  send_command_4_arg( MOTOR_1, SETACCEL, 0, 2, ret );  
  send_command_4_arg( MOTOR_1, SETDECEL, 0, 2, ret );  


  //update values
  send_command( MOTOR_0, UPDATE, 2, ret );
  send_command( MOTOR_1, UPDATE, 2, ret );

	_last_ltics = 0;
	_last_rtics = 0;
	
	return 0;
}

int 
ER::Setup()
{
  struct termios term;
  int flags;
  //int ltics,rtics,lvel,rvel;

  this->_px = this->_py = this->_pa = 0.0;
  this->_odom_initialized = false;

  printf("Evolution Robotics evolution_rcm connection initializing (%s)...\n", _serial_port);
  fflush(stdout);

  // open it.  non-blocking at first, in case there's no robot
  if((this->_fd = open(_serial_port, O_RDWR | O_NONBLOCK, S_IRUSR | S_IWUSR )) < 0 )
  {
    printf("open() failed: %s\n", strerror(errno));
    return(-1);
  }  
 
  if(tcgetattr(this->_fd, &term) < 0 )
  {
    printf("tcgetattr() failed: %s\n", strerror(errno));
    close(this->_fd);
    this->_fd = -1;
    return(-1);
  }

  cfmakeraw( &term );
  cfsetispeed(&term, B230400);
  cfsetospeed(&term, B230400);
  if(tcsetattr(this->_fd, TCSADRAIN, &term) < 0 )
  {
    printf("tcsetattr() failed: %s\n", strerror(errno));
    close(this->_fd);
    this->_fd = -1;
    return(-1);
  }

  _fd_blocking = false;
  if(InitRobot() < 0)
  {
    printf("failed to initialize robot\n");
    close(this->_fd);
    this->_fd = -1;
    return(-1);
  }

	/* ok, we got data, so now set NONBLOCK, and continue */
	if((flags = fcntl(this->_fd, F_GETFL)) < 0)
	{
		printf("fcntl() failed: %s\n", strerror(errno));
		close(this->_fd);
		this->_fd = -1;
		return(-1);
	}

	if(fcntl(this->_fd, F_SETFL, flags ^ O_NONBLOCK) < 0)
	{
		printf("fcntl() failed: %s\n", strerror(errno));
		close(this->_fd);
		this->_fd = -1;
		return(-1);
	}
	_fd_blocking = true;

	/*  This might be a good time to reset the odometry values */
	if( InitOdom() < 0 ) {
		printf("InitOdom failed\n" );
		close(this->_fd);
		this->_fd = -1;
		return -1;
	}

	// start the thread to talk with the robot
	this->StartThread();

	return(0);
}


int
ER::Shutdown()
{

  if(this->_fd == -1)
    return(0);

  StopThread();

  // the robot is stopped by the thread cleanup function StopRobot(), which
  // is called as a result of the above StopThread()
  
  if(SetVelocity(0,0) < 0)
    printf("failed to stop robot while shutting down\n");
  

  usleep(ER_DELAY_US);

  if(close(this->_fd))
    printf("close() failed:%s\n",strerror(errno));
  this->_fd = -1;
  if( _debug )
    printf("ER has been shutdown\n\n");
  
  return(0);
}
void
ER::Stop( int StopMode ) {
	
	unsigned char ret[8];
	
	printf( "Stop\n" );
	/* Start with motor 0*/
	_stopped = true;
  if( StopMode == FULL_STOP )
  {
    /* motor 0 */
    send_command_2_arg( MOTOR_0, RESETEVENTSTATUS, 0x0000, 2, ret );
    send_command_2_arg( MOTOR_0, SETMOTORCMD, 0x0000, 2, ret );
    send_command_2_arg( MOTOR_0, SETPROFILEMODE, 0x0001, 2, ret );
    send_command_2_arg( MOTOR_0, SETSTOPMODE, 0x0001, 2, ret );
    send_command_4_arg( MOTOR_0, SETVEL, 0, 2, ret );
    send_command_4_arg( MOTOR_0, SETACCEL, 0, 2, ret );
    send_command_4_arg( MOTOR_0, SETDECEL, 0, 2, ret );
    send_command( MOTOR_0, UPDATE, 2, ret );
    send_command( MOTOR_0, RESET, 2, ret );

    /* motor 1 */
    send_command_2_arg( MOTOR_1, RESETEVENTSTATUS, 0x0000, 2, ret );
    send_command_2_arg( MOTOR_1, SETMOTORCMD, 0x0000, 2, ret );
    send_command_2_arg( MOTOR_1, SETPROFILEMODE, 0x0001, 2, ret );
    send_command_2_arg( MOTOR_1, SETSTOPMODE, 0x0001, 2, ret );
    send_command_4_arg( MOTOR_1, SETVEL, 0, 2, ret );
    send_command_4_arg( MOTOR_1, SETACCEL, 0, 2, ret );
    send_command_4_arg( MOTOR_1, SETDECEL, 0, 2, ret );
    send_command( MOTOR_1, UPDATE, 2, ret );
    send_command( MOTOR_1, RESET, 2, ret );
  }
  else
  {
    /* motor 0 */
    send_command_2_arg( MOTOR_0, RESETEVENTSTATUS, 0x0700, 2, ret );
    send_command_4_arg( MOTOR_0, SETVEL, 0, 2, ret );
    send_command( MOTOR_0, UPDATE, 2, ret );
    send_command( MOTOR_0, RESET, 2, ret );

    /* motor 1 */
    send_command_2_arg( MOTOR_1, RESETEVENTSTATUS, 0x0700, 2, ret );
    send_command_4_arg( MOTOR_1, SETVEL, 0, 2, ret );
    send_command( MOTOR_1, UPDATE, 2, ret );
    send_command( MOTOR_1, RESET, 2, ret );
  
  }
	
}

////////////////////
// periodic fcns
////////////////////


int
ER::GetOdom(int *ltics, int *rtics, int *lvel, int *rvel)
{

	unsigned char ret[6];

	/* motor 0 */
  send_command( MOTOR_0, GETCMDPOS, 6, ret );
	*ltics = _motor_0_dir * BytesToInt32(&(ret[2]));

	/* motor 1 */
  send_command( MOTOR_1, GETCMDPOS, 6, ret );
	*rtics = _motor_1_dir * BytesToInt32(&(ret[2]));

	/* hmmm, what to do here ??? */
	/*
	index += 4;
	*rvel = BytesToInt32(buf+index);
	index += 4;
	*lvel = BytesToInt32(buf+index);
	*/

  if( _debug )
  {
    printf("ltics: %d rtics: %d\n", *ltics, *rtics);
  } 

	return(0);
}

int
ER::GetBatteryVoltage(int* voltage)
{
	
	unsigned char ret[4];
	
  send_command_2_arg( MOTOR_1, READANALOG, 0x0001, 6, ret );

  if( _debug )
   	printf( "voltage?: %f\n", (float) BytesToFloat(&(ret[2])) );
  
  //yeah and do something with this voltage???
  
	return(0);
}

int
ER::GetRangeSensor(int s, float * val )
{
	
	unsigned char ret[6];

  send_command_2_arg( s / 8, READANALOG, s % 8, 6, ret );

	/* this is definately wrong, need to fix this */
	float range = (float) BytesToFloat( &(ret[2]) );

  if( _debug )
  	printf( "sensor value?: %d\n", s );

	val = &range;

	return 0;
}


int
ER::SetVelocity(double lvel, double rvel)
{

	unsigned char ret[8];
  if( _debug )
	printf( "lvel: %f rvel: %f\n", lvel, rvel );

  send_command( MOTOR_0, GETEVENTSTATUS, 4, ret );
  
  if( _stopped )
  {
    send_command_2_arg( MOTOR_0, RESETEVENTSTATUS, 0x0300, 2, ret );
    send_command_2_arg( MOTOR_0, SETMOTORCMD, 0x6590, 2, ret );
    send_command_2_arg( MOTOR_0, SETPROFILEMODE, 0x0001, 2, ret );
  }
  else
  {
    send_command_2_arg( MOTOR_0, RESETEVENTSTATUS, 0x0700, 2, ret );
  
  }
  
  SpeedCommand( MOTOR_0, lvel, _motor_0_dir );
  
  send_command_4_arg( MOTOR_0, SETACCEL, 0x0000007E, 2, ret );


  send_command( MOTOR_1, GETEVENTSTATUS, 4, ret );
  if( _stopped )
  {
    send_command_2_arg( MOTOR_1, RESETEVENTSTATUS, 0x0300, 2, ret );
    send_command_2_arg( MOTOR_1, SETMOTORCMD, 0x6590, 2, ret );
    send_command_2_arg( MOTOR_1, SETPROFILEMODE, 0x0001, 2, ret );
  }
  else
  {
    send_command_2_arg( MOTOR_1, RESETEVENTSTATUS, 0x0700, 2, ret );
  }
  
  SpeedCommand( MOTOR_1, rvel, _motor_1_dir );
  send_command_4_arg( MOTOR_1, SETACCEL, 0x0000007E, 2, ret );


  send_command( MOTOR_0, UPDATE, 2, ret );
  send_command( MOTOR_1, UPDATE, 2, ret );

	_stopped = false;
  return 0;
}


void 
ER::Main()
{

	player_position2d_data_t data;
	int rtics, ltics, lvel, rvel;
	double lvel_mps, rvel_mps;
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);

	// push a pthread cleanup function that stops the robot
	pthread_cleanup_push(StopRobot,this);

	for(;;)
	{
		pthread_testcancel();
		ProcessMessages();
    
    	// handle pending config requests
	    //this->HandleConfig();

		//PLAYER_WARN("Done with handle config" );

		/* position command */

		//this->GetCommand();

		//PLAYER_WARN("Done with get Command config" );
		/* get battery voltage */

/*

		TODO:
			get hex to voltage to percentage conversion function

    	int volt;
	    if(GetBatteryVoltage(&volt) < 0) {
			PLAYER_WARN("failed to get voltage");
		}
		else printf("volt: %d\n", volt);

*/
	
		/* Get the 13 range sensor values */

/*

		TODO: 	read from config file to judge where the IRS are and what
			type they are
			
				hex => raw => distance for each type

		float val;
		for( int i = 0; i < 13; i++ ) {
			if( GetRangeSensor( i, &val ) < 0) {
				PLAYER_WARN("failed to get range sensor");
			}
		} 

*/

		/* get the odometry values */
		GetOdom( &ltics, &rtics, &lvel, &rvel );
		UpdateOdom( ltics, rtics );


		double tmp_angle;
		data.pos.px = this->_px;
		data.pos.py = this->_py;
		if(this->_pa < 0)
			tmp_angle = this->_pa + 2*M_PI;
		else
			tmp_angle = this->_pa;

		data.pos.pa = tmp_angle;

		data.vel.py = 0;
		lvel_mps = lvel * ER_MPS_PER_TICK;
		rvel_mps = rvel * ER_MPS_PER_TICK;
		data.vel.px = (lvel_mps + rvel_mps) / 2.0;
		data.vel.pa = (rvel_mps-lvel_mps) / 
                                             _axle_length;

                Publish(position_id, NULL, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, &data, sizeof(data));
//		PutData((unsigned char*)&data,sizeof(data),0,0);



	    usleep(ER_DELAY_US);

	} /* for */
	pthread_cleanup_pop(1);
  
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int ER::ProcessMessage(MessageQueue * resp_queue,
                               player_msghdr * hdr,
                               void * data)
{
  assert(hdr);
  assert(data);
	
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_GET_GEOM, position_id))
  {
  	player_position2d_geom_t geom;

    //TODO : get values from somewhere.
	geom.pose.px = -0.1;//htons((short) (-100));
	geom.pose.py = 0;//htons((short) (0));
	geom.pose.pa = 0;//htons((short) (0));
	geom.size.sw = 0.25;//htons((short) (250));
	geom.size.sl = 0.425;//htons((short) (425));    
    Publish(position_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_GET_GEOM, &geom, sizeof(geom));	

    return 0;
  }
  
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_MOTOR_POWER, position_id))
  {
  	player_position2d_power_config_t * powercfg = reinterpret_cast<player_position2d_power_config_t *> (data);

    printf("got motor power req: %d\n", powercfg->state);
	if(ChangeMotorState(powercfg->state) < 0)
	  Publish(position_id, resp_queue, PLAYER_MSGTYPE_RESP_NACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
	else
	  Publish(position_id, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
    return 0;
  }

  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL, position_id))
  {
  	player_position2d_cmd_vel_t & position_cmd = *reinterpret_cast<player_position2d_cmd_vel_t *> (data);

	double rotational_term;
	double command_rvel, command_lvel;
	double final_lvel = 0, final_rvel = 0;
	double last_final_lvel = 0, last_final_rvel = 0;

	// convert (tv,rv) to (lv,rv) and send to robot
	rotational_term = (position_cmd.vel.pa * M_PI / 180.0) * (_axle_length*1000.0) / 2.0;
	command_rvel = (position_cmd.vel.px) + rotational_term;
	command_lvel = (position_cmd.vel.px) - rotational_term;
    
//	printf( "position_cmd.xspeed: %d position_cmd.yawspeed: %d\n", position_cmd.xspeed, position_cmd.yawspeed );
	
	// sanity check on per-wheel speeds
	if(fabs(command_lvel) > ER_MAX_WHEELSPEED) {
		if(command_lvel > 0) {
			command_lvel = ER_MAX_WHEELSPEED;
			command_rvel *= ER_MAX_WHEELSPEED/command_lvel;
		}
		else {
			command_lvel = - ER_MAX_WHEELSPEED;
			command_rvel *= -ER_MAX_WHEELSPEED/command_lvel;
		}
	}
	if(fabs(command_rvel) > ER_MAX_WHEELSPEED) {
		if(command_rvel > 0) {
			command_rvel = ER_MAX_WHEELSPEED;
			command_lvel *= ER_MAX_WHEELSPEED/command_rvel;
		}
		else {
			command_rvel = - ER_MAX_WHEELSPEED;
			command_lvel *= -ER_MAX_WHEELSPEED/command_rvel;
		}
	}

	final_lvel = command_lvel;
	final_rvel = command_rvel;

  if( _debug )
    printf( "final_lvel: %f, final_rvel: %f\n", final_lvel, final_rvel );


	

	// TODO: do this min threshold smarter, to preserve desired travel 
	// direction

	if((final_lvel != last_final_lvel) ||
		(final_rvel != last_final_rvel)) {
		if( final_lvel * last_final_lvel < 0 || final_rvel * last_final_rvel < 0 ) {
//				PLAYER_WARN( "Changing motor direction, soft stop\n" );
			SetVelocity(0,0);
		}
		if(SetVelocity(final_lvel/10.0,final_rvel/10.0) < 0) {
			printf("failed to set velocity\n");
			pthread_exit(NULL);
		}
		if( (int) position_cmd.vel.px == 0 && (int) position_cmd.vel.pa == 0 )
		{
			printf( "STOP\n" );
			Stop( FULL_STOP+1 );			
		}

		last_final_lvel = final_lvel;
		last_final_rvel = final_rvel;
		MotorSpeed();
	}


    return 0;
  }
  
  return -1;
}



////////////////////
// util fcns
////////////////////

void
ER::MotorSpeed()
{
	unsigned char ret[8];

  send_command( MOTOR_0, GETEVENTSTATUS, 4, ret );
  send_command_2_arg( MOTOR_0, RESETEVENTSTATUS, 0x0700, 2, ret );
  send_command_4_arg( MOTOR_0, SETACCEL, 0x0000007A, 2, ret );
  send_command( MOTOR_1, GETEVENTSTATUS, 4, ret );
  send_command_2_arg( MOTOR_1, RESETEVENTSTATUS, 0x0700, 2, ret );
  send_command_4_arg( MOTOR_1, SETACCEL, 0x0000007A, 2, ret );

  send_command( MOTOR_0, UPDATE, 2, ret );
  send_command( MOTOR_1, UPDATE, 2, ret );
}

void
ER::SpeedCommand( unsigned char address, double speed, int dir ) {
	
  unsigned char ret[2];
  
	int whole = dir * (int) (speed * 16819.8);

  send_command_4_arg( address, SETVEL, whole, 2, ret );

//	printf( "speed: %f checksum: 0x%02x value: 0x%08x\n", speed, cmd[1], whole );
	
}

int 
ER::BytesToInt32(unsigned char *ptr)
{
  unsigned char char0,char1,char2,char3;
  int data = 0;

  char0 = ptr[3];
  char1 = ptr[2];
  char2 = ptr[1];
  char3 = ptr[0];

  data |=  ((int)char0)        & 0x000000FF;
  data |= (((int)char1) << 8)  & 0x0000FF00;
  data |= (((int)char2) << 16) & 0x00FF0000;
  data |= (((int)char3) << 24) & 0xFF000000;

  //this could just be ntohl

  return data;
}

float 
ER::BytesToFloat(unsigned char *ptr)
{
//  unsigned char char0,char1,char2,char3;
  float data = 0;

	sscanf( (const char *) ptr, "%f", &data );

  return data;
}

int 
ER::ComputeTickDiff(int from, int to) 
{
  int diff1, diff2;

  // find difference in two directions and pick shortest
  if(to > from) 
  {
    diff1 = to - from;
    diff2 = (-ER_MAX_TICKS - from) + (to - ER_MAX_TICKS);
  }
  else 
  {
    diff1 = to - from;
    diff2 = (from - ER_MAX_TICKS) + (-ER_MAX_TICKS - to);
  }

  if(abs(diff1) < abs(diff2)) 
    return(diff1);
  else
    return(diff2);

	return 0;
}

void
ER::UpdateOdom(int ltics, int rtics)
{
	
	int ltics_delta, rtics_delta;
	double l_delta, r_delta, a_delta, d_delta;
	int max_tics;
	static struct timeval lasttime;
	struct timeval currtime;
	double timediff;

	if(!this->_odom_initialized)
	{
		this->_last_ltics = ltics;
		this->_last_rtics = rtics;
		gettimeofday(&lasttime,NULL);
		this->_odom_initialized = true;
		return;
	}
  
//	ltics_delta = ComputeTickDiff(last_ltics,ltics);
//	rtics_delta = ComputeTickDiff(last_rtics,rtics);

	ltics_delta = ltics - this->_last_ltics;
	rtics_delta = rtics - this->_last_rtics;

  // mysterious rollover code borrowed from CARMEN
/* 
  if(ltics_delta > SHRT_MAX/2)
    ltics_delta += SHRT_MIN;
  if(ltics_delta < -SHRT_MIN/2)
    ltics_delta -= SHRT_MIN;
  if(rtics_delta > SHRT_MAX/2)
    rtics_delta += SHRT_MIN;
  if(rtics_delta < -SHRT_MIN/2)
    rtics_delta -= SHRT_MIN;
*/

  gettimeofday(&currtime,NULL);
  timediff = (currtime.tv_sec + currtime.tv_usec/1e6)-
             (lasttime.tv_sec + lasttime.tv_usec/1e6);
  max_tics = (int)rint(ER_MAX_WHEELSPEED / ER_M_PER_TICK / timediff);
  lasttime = currtime;
	
	if( _debug ) {
	  printf("ltics: %d\trtics: %d\n", ltics,rtics);
	  printf("ldelt: %d\trdelt: %d\n", ltics_delta, rtics_delta);
	}
  //printf("maxtics: %d\n", max_tics);

  if(abs(ltics_delta) > max_tics || abs(rtics_delta) > max_tics)
  {
    printf("Invalid odometry change (too big); ignoring\n");
    return;
  }

  l_delta = ltics_delta * ER_M_PER_TICK;
  r_delta = rtics_delta * ER_M_PER_TICK;


  a_delta = (r_delta - l_delta) / ( _axle_length );
  d_delta = (l_delta + r_delta) / 2.0;

  this->_px += d_delta * cos(this->_pa + (a_delta / 2));
  this->_py += d_delta * sin(this->_pa + (a_delta / 2));
  this->_pa += a_delta;
  this->_pa = NORMALIZE(this->_pa);
  
	if( _debug ) {
		printf("er: pose: %f,%f,%f\n", this->_px,this->_py,this->_pa * 180.0 / M_PI);
	}
  this->_last_ltics = ltics;
  this->_last_rtics = rtics;
}


int 
ER::ChangeMotorState(int state)
{
/*
  unsigned char buf[1];
  if(state)
    buf[0] = TROGDOR_ENABLE_VEL_CONTROL;
  else
    buf[0] = TROGDOR_DISABLE_VEL_CONTROL;
  return(WriteBuf(buf,sizeof(buf)));
*/
	return 0;
}

static void
StopRobot(void* erdev)
{
  ER* er = (ER*)erdev;

//  if(er->Stop( FULL_STOP ) < 0)
//    PLAYER_ERROR("failed to stop robot on thread exit");

	er->Stop(FULL_STOP );
}
