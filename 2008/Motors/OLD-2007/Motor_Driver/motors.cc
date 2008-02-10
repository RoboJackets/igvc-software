#include "motors.h"
Driver* Motors_Player::Motors_Player_Init(ConfigFile* cf, int section)
{
	return( (Driver*)(new Motors_Player(cf, section)) );
}

void Motors_Player::Motors_Player_Register(DriverTable* table)
{
	table->AddDriver("motors",  Motors_Player_Init);
	return;
}

Motors_Player::Motors_Player(ConfigFile* cf, int section) 
 : Driver(cf, section, TRUE, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION2D_CODE)
{
	/* Read parameters from the configuration file */
	const char* cpMotorPort = cf->ReadString(section, "port", DEFAULT_PORT);
	double pGain = cf->ReadFloat(section, "PGain", DEFAULT_PGAIN);
	double iGain = cf->ReadFloat(section, "IGain", DEFAULT_IGAIN);
	double dGain = cf->ReadFloat(section, "DGain", DEFAULT_DGAIN);
	this->dVelocityScale = cf->ReadFloat(section, "Velocity Scale", DEFAULT_VELOCITY_SCALE);
	//dMaxSpeed
	//dMaxAccel
	
	this->driver = new Motors(cpMotorPort, pGain, iGain, dGain);

	/* Only recieve messages related to controlling the Motors_Player */
	//SetFilter();

	/* Sets PLAYER_POSITION2D_CMD_POS messages (commands to move to a position)
	 * to not be replaced by new messages of the same type.  All other command
	 * messages are replaced by newer ones (could be set otherwise if needed).
	 */
	InQueue->AddReplaceRule(device_addr, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_POS, FALSE);
}

int Motors_Player::Setup()
{
	if (!driver->Connect()) { return -1; }
	StartThread();
	return(0);
}

int Motors_Player::Shutdown()
{
	StopThread();
	if (!driver->Disconnect()) { return -1; }
	return(0);
}

int Motors_Player::ProcessMessage(MessageQueue* resp_queue, player_msghdr* hdr, void* data) 
{
	if(hdr->type == PLAYER_MSGTYPE_REQ)
	{
		/* Turn the motors on and off - to be used in emergency situations when the robot
		 * needs to avoid running into something.
		 */
		if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_MOTOR_POWER,
					device_addr) )
		{
			// TODO: implement
			return 0;
		}
		/* Change the way velcoity commands are sent */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_VELOCITY_MODE,
					device_addr) )
		{
			// TODO: implement
			return 0;
		}
		/* Choose between sending velocity commands or position commands */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_POSITION_MODE,
					device_addr) )
		{
			// TODO: implement
			return 0;
		}
		/* Set velocity PID parameters */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_SPEED_PID,
					device_addr) )
		{
			return SetMotorPID((player_position2d_speed_pid_req_t *)data);
		}

		/* Set maximum speed and accelleration */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_REQ,
					PLAYER_POSITION2D_REQ_SPEED_PROF,
					device_addr))
		{
			// TODO: implement
			return 0;
		}
		else
		{
			return(-1);
		}
	}
	else if(hdr->type == PLAYER_MSGTYPE_CMD)
	{
		/* This is a set of motor speeds that should be outputed */
		if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_CMD,
					PLAYER_POSITION2D_CMD_VEL,
					device_addr) )
		{
			return( SetMotorSpeeds((player_position2d_cmd_vel_t *)data) );
		}
		/* This is a position to which the robot should move */
		else if( Message::MatchMessage(hdr,
					PLAYER_MSGTYPE_CMD,
					PLAYER_POSITION2D_CMD_POS,
					device_addr) )
		{
			// TODO: implement
			return 0;
		}
		else
		{
			return(-1);
		}
	}
	else
	{
		return(-1);
	}
}

void Motors_Player::Main()
{
	for(;;)
	{
		/* Test if the main thread should cancel */
		pthread_testcancel();

		/* Run ProcessMessage() for each message in the queqe */
		ProcessMessages();
	}
}

int Motors_Player::SetMotorPID(player_position2d_speed_pid_req_t* data)
{
	driver->SetPID(data->kp, data->ki, data->kd);
	return 0;
}

int Motors_Player::SetMotorSpeeds(player_position2d_cmd_vel_t* stVelocity)
{
	//printf("LeftVel command: %f\n", stVelocity->vel.px);
	//printf("RightVel command: %f\n\n", stVelocity->vel.pa);
	
	/* Scale the motor velocities, limit them to vaild values, and store them in the
	 * correct form to transmit to the motor controller.
	 */
	int iScaledLeftVel = (int)(dVelocityScale * stVelocity->vel.px);
	int iScaledRightVel = (int)(dVelocityScale * stVelocity->vel.pa);
	
	return driver->SetSpeeds(iScaledLeftVel, iScaledRightVel) ? 0 : -1;
}

/* Extra stuff for building a shared object.
 * need the extern to avoid C++ name-mangling
 */
extern "C" {
	int player_driver_init(DriverTable* table)
	{
		puts("motors driver initializing");
		Motors_Player::Motors_Player_Register(table);
		puts("motors driver done");
		return(0);
	}
}
