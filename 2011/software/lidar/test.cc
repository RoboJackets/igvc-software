
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

		std::deque< boost::tuple<float,float> > lines;
		a.findLinearRuns(lines);

		boost::tuple<float,float> longest;
		NAV200::getLongestRun(lines, longest);

		std::cout << "idx: " << longest;

		usleep(1e6);
	}
	return 0;

}
