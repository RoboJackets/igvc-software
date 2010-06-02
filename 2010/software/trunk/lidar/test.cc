
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

	//NAV200 a;
	float domains[NAV200::Num_Points] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28};
	float ranges[NAV200::Num_Points] = {1,2,3,4,5,0,2,4,6,8,0,3,6,9,0,4,9,16,25, -2,-4,-6, .1, .3, .5, 5, 15, 25};
	boost::tuple<float,float> testcoord[NAV200::Num_Points];

	for(int i = 0; i < NAV200::Num_Points; i++)
	{
		testcoord[i].get<0>() = domains[i];
		testcoord[i].get<1>() = ranges[i];
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
	std::cout << "ranges: ";printarray(ranges, NAV200::Num_Points);
	std::cout << "rampmap: ";printarray(rampmap, NAV200::Num_Points);

	return 0;

}
