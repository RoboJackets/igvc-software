
#include "gyro.hpp"
#include "nmea.hpp"

#include <iostream>

gyro::gyro() : running(false), gyro_port(io_service), queue_len(50)
{
	
}
gyro::~gyro()
{
	stop();
}

void gyro::start()
{
	running = true;
	iothread = boost::thread(boost::bind(&gyro::gyro_comm, this));
}

void gyro::stop()
{
	running = false;
	iothread.join();
}

bool gyro::get_last_state(gyroState& state)
{
	boost::mutex::scoped_lock lock(state_queue_mutex);

	if(state_queue.empty())
	{
		std::cerr << "gyro queue empty!" << std::endl;
		return false;
	}

	struct timeval now;
	gettimeofday(&now, NULL);
	time_t secDelta = difftime(now.tv_sec, state_queue.back().laptoptime.tv_sec);
	suseconds_t usecDelta = now.tv_usec - state_queue.back().laptoptime.tv_usec;
	double delta = double(secDelta) + 1e-6*double(usecDelta);
	if(delta > .5)
	{
		std::cerr << "gyro Marker Out Of Date!" << std::endl;
		return false;
	}

	state = state_queue.back();
	return true;
}

bool gyro::open(const char* port, size_t baud)
{
	m_port = port;
	m_baud = baud;

	try
	{
		gyro_port.open(port);
	}
	catch(...)
	{
		std::cerr << "Failed to open serial port" << std::endl;
		return false;
	}

	if(!gyro_port.is_open())
	{
		return false;
	}
	
	try
	{
		gyro_port.set_option(boost::asio::serial_port_base::baud_rate(baud));
		gyro_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
		gyro_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		gyro_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	}
	catch(...)
	{
		return false;
	}
	
	m_connected = true;

	return true;
}

void gyro::close()
{
	gyro_port.close();
	m_connected = false;
}

void gyro::handle_serial_read_timer(const boost::system::error_code& ec)
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

	gyro_port.cancel();
	//reconnect();
}

void gyro::handle_serial_read(const boost::system::error_code& ec, size_t len, boost::asio::deadline_timer& timeout)
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
		std::cerr << "Error parsing gyro packet!" << std::endl;
		return;
	}

	try
	{
		gyroState state;
		if(nmea::decodeRPY(line, state))
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
		std::cerr << "Error parsing gyro packet!" << std::endl;
	}
}

void gyro::gyro_comm()
{
	while(running)
	{
		boost::asio::deadline_timer timeout(io_service);
		timeout.expires_from_now(boost::posix_time::milliseconds(2e3));
		try
		{
			boost::asio::async_read_until(gyro_port, comm_buffer, '\n', boost::bind(&gyro::handle_serial_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred, boost::ref(timeout)));

			timeout.async_wait(boost::bind(&gyro::handle_serial_read_timer, this, boost::asio::placeholders::error));

			io_service.run();
			io_service.reset();
		}
		catch(...)
		{
			//reconnect();
		}
	}
}

void gyro::reconnect()
{
	close();
	usleep(1e6);
	open(m_port.c_str(), m_baud);
}

bool gyro::parse_message(const std::string& line, gyroState& state)
{
	return nmea::decodeRPY(line, state);
}


