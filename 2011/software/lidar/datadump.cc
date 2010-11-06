
#include "NAV200.hpp"

#include <iostream>

#include "boost/tuple/tuple.hpp"
#include "boost/tuple/tuple_io.hpp"
#include "boost/foreach.hpp"

typedef boost::tuple<float,float> range;

template<typename T>
void printarray(T* v, size_t len)
{
	std::cout << "<" << v[0] << ", ";
	for(size_t i = 1; i < (len-1); i++)
	{
		std::cout << v[i] << ", ";
	}
	std::cout << v[len-1] << ">\n";
}

template<typename T>
void printarray(T* v, size_t start, size_t stop)
{
	std::cout << "<" << v[start] << ", ";
	for(size_t i = start+1; i < stop; i++)
	{
		std::cout << v[i] << ", ";
	}
	std::cout << v[stop] << ">\n";
}

int main()
{

	NAV200 a;


	for(;;)
	{
		if(!a.read())
		{
			std::cout << "could not read from lidar" << std::endl;
			usleep(1e5);
			continue;
		}

		std::cout << "float theta[NAV200::Num_Points] = {";
		for(int i = 0; i < (NAV200::Num_Points-2); i++)
		{
			std::cout << a.theta[i] << ", ";
		}
		std::cout << a.theta[NAV200::Num_Points-1] << "}\n";

		std::cout << "float radius[NAV200::Num_Points] = {";
		for(int i = 0; i < (NAV200::Num_Points-2); i++)
		{
			std::cout << a.radius[i] << ", ";
		}
		std::cout << a.radius[NAV200::Num_Points-1] << "}\n\n";

		//usleep(1e6);
		usleep(2e5);
	}

	return 0;

}
