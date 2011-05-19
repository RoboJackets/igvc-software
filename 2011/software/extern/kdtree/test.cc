

#include <iostream>

#include <boost/random.hpp>

#include "kdtree.h"

int main()
{

	struct kdtree* kdt = kd_create(2);

	boost::mt19937 rng;
	boost::uniform_real<> unit(0.0, 1.0);
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(rng, unit);

	double pos[2];
	for(int i = 0; i < 1e6; i++)
	{
		pos[0] = die();
		pos[1] = die();
		int res =  kd_insert(kdt, pos, NULL);
	}

	//struct kdres *kd_nearest_range(struct kdtree *tree, const double *pos, double range);
	pos[0] = 0;
	pos[1] = 0;
	struct kdres* res = kd_nearest_range(kdt, pos, .1);

	if(kd_res_size(res) > 0)
	{
		do
		{
			double point[2];
			void* ptr = kd_res_item(res, point);
			std::cout << "(" << point[0] << ", " << point[1] << ")" << std::endl;
		} while(kd_res_next(res));
	}
	kd_res_free(res);
	kd_free(kdt);
}
