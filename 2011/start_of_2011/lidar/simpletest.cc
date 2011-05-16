
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

	if(!a.read())
	{
		std::cout << "could not read from lidar" << std::endl;
		return -1;
	}

	float domain[NAV200::Num_Points];
	float range[NAV200::Num_Points];

	for(int i = 0; i < NAV200::Num_Points; i++)
	{
		domain[i] = a.coord[i].get<0>();
		range[i] = a.coord[i].get<1>();
	}

	std::cout << "ranges: ";printarray(range, NAV200::Num_Points);

	return 0;

}
