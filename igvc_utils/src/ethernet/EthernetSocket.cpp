#include <igvc_utils/EthernetSocket.h>
#include <iostream>

using boost::asio::ip::tcp;

EthernetSocket::EthernetSocket(std::string ip_addr, int port)
{
  // resolve all possible endpoints
  tcp::resolver resolver(io_service);
  tcp::resolver::query query(ip_addr, std::to_string(port));
  tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

  // look through endpoints and hit socket's connect() member function until
  // a successful TCP connection is established
  this->sock = igvc::make_unique<tcp::socket>(io_service);
  boost::asio::connect(*sock, endpoint_iterator);
}

EthernetSocket::~EthernetSocket()
{
  // shut down the TCP connection
  this->sock->shutdown(boost::asio::ip::tcp::socket::shutdown_send);
}

void EthernetSocket::sendMessage(std::string message)
{
  boost::system::error_code error;
  // Create boost buffer from string and send to TCP endpoint
  boost::asio::write(*sock, boost::asio::buffer(message, sizeof(message)), error);
}

std::string EthernetSocket::readMessage()
{
  // read data from TCP connection
  boost::array<char, 128> buf;
  boost::system::error_code error;

  size_t len = sock->read_some(boost::asio::buffer(buf), error);
  std::string reading(buf.begin(), buf.end());  // get string from buffer iter

  if (error == boost::asio::error::eof)
  {
    std::cout << "TCP Connection closed by peer: Disconnecting" << std::endl;
  }
  else if (error)
  {
    throw boost::system::system_error(error);
  }

  return reading;
}

std::string EthernetSocket::getIP()
{
  return sock->remote_endpoint().address().to_string();
}

int EthernetSocket::getPort()
{
  return static_cast<int>(sock->remote_endpoint().port());
}

std::string EthernetSocket::getBoostVersion()
{
  std::stringstream version;
  version << BOOST_VERSION / 100000 << "."      // major version
          << BOOST_VERSION / 100 % 1000 << "."  // minor version
          << BOOST_VERSION % 100;               // patch level

  return version.str();
}
