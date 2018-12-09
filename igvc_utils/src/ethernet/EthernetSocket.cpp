#include <igvc_utils/EthernetSocket.h>
#include <iostream>

EthernetSocket::EthernetSocket(std::string ip_addr, int port) :
    s(ioservice)
{
    boost::asio::ip::tcp::endpoint endpoint(
        boost::asio::ip::address::from_string(ip_addr),
        static_cast<unsigned short>(port));

    try
    {
        s.open(boost::asio::ip::tcp::v4());
    } catch (...) {
        throw std::runtime_error("Could not open connection: "
                + ip_addr + ", port: " + std::to_string(port));
    }

    if (!s.is_open())
    throw std::runtime_error("Could not open connection: "
            + ip_addr + ", port: " + std::to_string(port));
    // bind socket to an endpoint
    s.bind(endpoint);

    // connect the socket to the specified endpoint
    connected = true;
    try
    {
        s.connect(endpoint);
    } catch(...) {
        connected = false;
    }

    if (!connected)
    {
        throw std::runtime_error("Could not connect to ip address: "
                + ip_addr + ", port: " + std::to_string(port));
    }
}

bool EthernetSocket::isOpen()
{
    return s.is_open();
}

void EthernetSocket::write(std::string msg)
{
    if (s.is_open())
        s.send(boost::asio::buffer(msg.c_str(), msg.length()));
}

void EthernetSocket::write(char* buffer, int length)
{
    if (s.is_open())
        s.send(boost::asio::buffer(buffer, length));
}

void EthernetSocket::write(unsigned char* buffer, int length)
{
    if (s.is_open())
        s.send(boost::asio::buffer(buffer, length));
}

char EthernetSocket::read()
{
    if (!s.is_open())
        return -1;

    // read one character on the socket
    char in;
    try {
        s.receive(boost::asio::buffer(&in, 1));
    }
    catch (
        boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >&
            err)
    {
      s.shutdown(boost::asio::ip::tcp::socket::shutdown_send);
      throw std::runtime_error("Error reading socket");
    }
    return in;
}

char* EthernetSocket::read(int numBytes)
{
    if (!s.is_open())
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
