
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

bool gps::open(const char* port, size_t baud)
{
	m_port = port;
	m_baud = baud;

	try
	{
		gps_port.open(port);
	}
	catch(...)
	{
		std::cerr << "Failed to open serial port" << std::endl;
		return false;
	}

	if(!gps_port.is_open())
	{
		return false;
	}
	
	try
	{
		gps_port.set_option(boost::asio::serial_port_base::baud_rate(baud));
		//gps_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
		gps_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		gps_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	}
	catch(...)
	{
		return false;
	}
	
	m_connected = true;

	return true;
}

void gps::close()
{
	gps_port.close();
	m_connected = false;
}

void gps::handle_serial_read(const boost::system::error_code& ec, size_t len)
{
	readPending = false;
}

void gps::gps_comm()
{
	while(running)
	{
		std::string line;
		try
		{
			time_t t1 = time(NULL);
			readPending = true;
			boost::asio::async_read_until(gps_port, comm_buffer, '\n', boost::bind(&gps::handle_serial_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
			do
			{
				time_t t2 = time(NULL);

				if((t2 - t1) > SERIAL_TIMOUT_SEC)
				{
					std::cout << "serial timeout" << std::endl;
					gps_port.cancel();
					do
					{
					reconnect();
					} while(!m_connected);					
					continue;
				}

				io_service.poll();
				usleep(100);
			} while(readPending);
			io_service.reset();

		boost::asio::read_until(gps_port, comm_buffer, '\n');
		std::istream is(&comm_buffer);
		
		std::getline(is, line);
		}
		catch(...)
		{
			do
			{
				reconnect();
			} while(!m_connected);
		}

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
			std::cerr << "Reconnecting" << std::endl;
			do
			{
				reconnect();
			} while(!m_connected);
		}
	}
}

void gps::reconnect()
{
	close();
	usleep(1e6);
	open(m_port.c_str(), m_baud);
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


