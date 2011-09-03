
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

bool gps::get_last_state(GPSState& state)
{
	boost::mutex::scoped_lock lock(state_queue_mutex);

	if(state_queue.empty())
	{
		std::cerr << "GPS queue empty!" << std::endl;
		return false;
	}

	struct timeval now;
	gettimeofday(&now, NULL);
	time_t secDelta = difftime(now.tv_sec, state_queue.back().laptoptime.tv_sec);
	suseconds_t usecDelta = now.tv_usec - state_queue.back().laptoptime.tv_usec;
	double delta = double(secDelta) + 1e-6*double(usecDelta);
	if(delta > .5)
	{
		std::cerr << "GPS Marker Out Of Date!" << std::endl;
		return false;
	}



	if( (state_queue.back().qual != GPS_QUALITY_NON_DIFF) && (state_queue.back().qual != GPS_QUALITY_WAAS))
	{
		std::cerr << "GPS state bad - " << int(state_queue.back().qual) << std::endl;
		return false;
	}


	state = state_queue.back();
	return true;
}

bool gps::get_speed(double& speed)
{
	GPSState state;
	if(get_last_state(state))
	{
		speed = state.speedoverground;
		return true;
	}
	return false;
}

bool gps::get_heading(double& heading)
{
	GPSState state;
	if(get_last_state(state))
	{
		heading = state.courseoverground;
		return true;
	}
	return false;
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

void gps::handle_serial_read_timer(const boost::system::error_code& ec)
{
	if(ec)
	{
		return;
	}

	std::cerr << "Serial Timout" << std::endl;
{
	boost::mutex::scoped_lock lock(state_queue_mutex);	
	state_queue.clear();
}

	gps_port.cancel();
	reconnect();
}

void gps::handle_serial_read(const boost::system::error_code& ec, size_t len, boost::asio::deadline_timer& timeout)
{
	if(ec || (len == 0))
	{
		return;
	}
	timeout.cancel();//data rec, kill the timer

	std::string line;
	try
	{
		std::istream is(&comm_buffer);	
		std::getline(is, line);
	}
	catch(...)
	{
		std::cerr << "Error parsing GPS packet!" << std::endl;
		return;
	}

	try
	{
		GPSState state;
		//if(nmea::decodeGPGGA(line, state))
		if(nmea::decodeGPRMC(line, state))
		{
			gettimeofday(&state.laptoptime, NULL);
			boost::mutex::scoped_lock lock(state_queue_mutex);
	
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

void gps::gps_comm()
{
	while(running)
	{
		boost::asio::deadline_timer timeout(io_service);
		timeout.expires_from_now(boost::posix_time::milliseconds(2e3));
		try
		{
			boost::asio::async_read_until(gps_port, comm_buffer, '\n', boost::bind(&gps::handle_serial_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred, boost::ref(timeout)));

			timeout.async_wait(boost::bind(&gps::handle_serial_read_timer, this, boost::asio::placeholders::error));

			io_service.run();
			io_service.reset();
		}
		catch(...)
		{
			reconnect();
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


