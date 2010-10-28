
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
	//float domains[NAV200::Num_Points] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28};
	//float ranges[NAV200::Num_Points] = {1,2,3,4,5,0,2,4,6,8,0,3,6,9,0,4,9,16,25, -2,-4,-6, .1, .3, .5, 5, 15, 25};
	boost::tuple<float,float> testcoord[NAV200::Num_Points];

for(;;)
{
	if(!a.read())
	{
		std::cout << "could not read from lidar" << std::endl;
		usleep(1e5);
		continue;
	}

	std::cout << "x: [";
	for(int i = 0; i < (NAV200::Num_Points-2); i++)
	{
		std::cout << a.coord[i].get<0>() << ", ";
	}
	std::cout << a.coord[NAV200::Num_Points-1].get<0>() << "]\n";

	std::cout << "y: [";
	for(int i = 0; i < (NAV200::Num_Points-2); i++)
	{
		std::cout << a.coord[i].get<1>() << ", ";
	}
	std::cout << a.coord[NAV200::Num_Points-1].get<1>() << "]\n\n";

	//usleep(1e6);
	usleep(2e5);
}
	return 0;

}
