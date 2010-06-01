
#include "NAV200.hpp"

#include <iostream>
#include "boost/tuple/tuple.hpp"
#include "boost/foreach.hpp"

typedef boost::tuple<int,int> range;

int main()
{

	//NAV200 a;

	float ranges[NAV200::Num_Points] = {1,2,3,4,5,0,2,4,6,8,0,3,6,9,0,4,9,16,25, -2,-4,-6, .1, .3, .5, 5, 15, 25};

	std::list< boost::tuple<int,int> > lines;
	NAV200::findLinearRuns(ranges, lines);

	int rampmap[NAV200::Num_Points];//differ between linear runs with different slopes
	std::fill(rampmap, rampmap+NAV200::Num_Points, -1);
	int i = 0;
	BOOST_FOREACH(range& e, lines)
	{
		std::cout << "start: " << e.get<0>() << ", stop: " << e.get<1>() << std::endl;
		std::cout << "\t";
		//printarray(ranges, e.get<0>(), e.get<1>());
		std::fill(rampmap+e.get<0>(), rampmap+e.get<1>()+1, i);
		i++;
	}
	std::cout << "rampmap: ";//printarray(rampmap, NAV200::Num_Points);

	return 0;

}
