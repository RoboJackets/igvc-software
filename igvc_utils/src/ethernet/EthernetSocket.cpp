#include <igvc_utils/EthernetSocket.h>
#include <iostream>

EthernetSocket::EthernetSocket(std::string ip_addr, int port) : sock(ioservice)
{
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip_addr),
                                          static_cast<unsigned short>(port));

  try
  {
    sock.open(boost::asio::ip::tcp::v4());
  }
  catch (...)
  {
  }

  if (!sock.is_open())
    throw std::runtime_error("Could not open connection: " + ip_addr + ", port: " + std::to_string(port));

  // bind socket to an endpoint
  sock.bind(endpoint);

  // connect the socket to the specified endpoint
  connected = true;
  try
  {
    sock.connect(endpoint);
  }
  catch (...)
  {
    connected = false;
  }

  if (!connected)
  {
    throw std::runtime_error("Could not connect to ip address: " + ip_addr + ", port: " + std::to_string(port));
  }
}

EthernetSocket::~EthernetSocket()
{
  sock.shutdown(boost::asio::ip::tcp::socket::shutdown_send);
}

void EthernetSocket::write(std::string msg)
{
  if (sock.is_open())
    sock.send(boost::asio::buffer(msg.c_str(), msg.length()));
}

void EthernetSocket::write(char* buffer, int length)
{
  if (sock.is_open())
    sock.send(boost::asio::buffer(buffer, length));
}

void EthernetSocket::write(unsigned char* buffer, int length)
{
  if (sock.is_open())
    sock.send(boost::asio::buffer(buffer, length));
}

char EthernetSocket::read()
{
  if (!sock.is_open())
    return -1;

  // read one character on the socket
  char in;
  try
  {
    sock.receive(boost::asio::buffer(&in, 1));
  }
  catch (
      boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >&
          err)
  {
    sock.shutdown(boost::asio::ip::tcp::socket::shutdown_send);
    throw std::runtime_error("Error reading socket");
  }
  return in;
}

char* EthernetSocket::read(int numBytes)
{
  if (!sock.is_open())
    return (char*)"";

  char* bytes = new char[numBytes];
  for (int i = 0; i < numBytes; i++)
    bytes[i] = read();

  return bytes;
}

std::string EthernetSocket::readln()
{
  std::string line = "";
  while (true)
  {
    char in = read();
    if (in == '\n')
      return line;
    if (in != '\r')
      line = line + in;
  }
}

std::string EthernetSocket::getIPAddress()
{
  return this->ip_address.to_string();
}

int EthernetSocket::getPort()
{
  return static_cast<int>(this->sock.remote_endpoint().port());
}

bool EthernetSocket::isOpen()
{
  return sock.is_open();
}

void EthernetSocket::flush()
{
  sock.cancel();
}
