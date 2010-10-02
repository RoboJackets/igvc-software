
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
	boost::tuple<float,float> testcoord[NAV200::Num_Points];

	if(!a.read())
	{
		std::cout << "could not read from lidar" << std::endl;
		return -1;
	}

	std::deque< boost::tuple<float,float> > lines;
	NAV200::findLinearRuns(testcoord, lines);

	boost::tuple<float,float> longest;
	NAV200::getLongestRun(lines, longest);

	std::cout << "longest run: " << longest << std::endl;

	int rampmap[NAV200::Num_Points];//differ between linear runs with different slopes
	std::fill(rampmap, rampmap+NAV200::Num_Points, -1);
	int i = 0;
	BOOST_FOREACH(range& e, lines)
	{
		std::cout << "start: " << e.get<0>() << ", stop: " << e.get<1>() << std::endl;
		std::cout << "\t";
		//printarray(ranges, e.get<0>(), e.get<1>());
		//std::fill(rampmap+e.get<0>(), rampmap+e.get<1>()+1, i);
		i++;
	}
	//std::cout << "ranges: ";printarray(ranges, NAV200::Num_Points);
	std::cout << "rampmap: ";printarray(rampmap, NAV200::Num_Points);

	return 0;

}
