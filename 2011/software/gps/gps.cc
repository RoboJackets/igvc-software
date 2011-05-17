
#include "gps.hpp"
#include "nmea.hpp"

#include <iostream>

gps::gps() : running(false), gps_port(io_service), queue_len(50)
{
	
}
gps::~gps()
{
	stop();
}

void gps::start()
{
	running = true;
	iothread = boost::thread(boost::bind(&gps::gps_comm, this));
}

void gps::stop()
{
	running = false;
	iothread.join();
}

bool gps::get_last_pos(GPSState& state)
{
	boost::mutex::scoped_lock lock(state_queue_mutex);

	if(state_queue.empty())
	{
		return false;
	}

/*
	if(fabsf(state_queue.back().utc_time - now) > .5)
	{
		return false;
	}
*/

/*
	if( (state_queue.back().qual != GPS_QUALITY_NON_DIFF) && (state_queue.back().qual != GPS_QUALITY_WAAS))
	{
		return false;
	}
*/

	state = state_queue.back();
	return true;
}

bool gps::open(char* port, size_t baud)
{
	gps_port.open(port);

	if(!gps_port.is_open())
	{
		return false;
	}
	
	gps_port.set_option(boost::asio::serial_port_base::baud_rate(baud));
	//gps_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
	gps_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	gps_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

	return true;
}

void gps::gps_comm()
{
	while(running)
	{
		boost::asio::read_until(gps_port, comm_buffer, '\n');
		std::istream is(&comm_buffer);
		std::string line;
		std::getline(is, line);

		try
		{
		GPSState state;
		if(nmea::decodeGPGGA(line, state))
		{
			state_queue.push_back(state);
			if(state_queue.size() > queue_len)
			{
				state_queue.pop_front();
			}
		}
		}
		catch(...)
		{
			std::cerr << "Error parsing GPS packet!" << std::endl;
		}
	}
}

bool gps::parse_message(const std::string& line, GPSState& state)
{
	if(nmea::decodeGPGGA(line, state))
	{
		return true;
	}
	if(nmea::decodeGPRMC(line, state))
	{
		return true;
	}
	if(nmea::decodeGPRMT(line))
	{
		return true;
	}
	if(nmea::decodeGPGSA(line))
	{
		return true;
	}
	if(nmea::decodeGPGSV(line))
	{
		return true;
	}
	return false;
}


