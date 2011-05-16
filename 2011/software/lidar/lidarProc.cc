#include "NAV200.hpp"

#include "lidarProc.hpp"
#include "finiteDiff.hpp"

#include "boost/foreach.hpp"

#include "opencv/cv.h"

#include <iostream>

namespace lidarProc
{

/*
bool findLinearRuns(const float* theta, const float* radius, const size_t len, const double zero_tol, std::deque< boost::tuple<size_t,size_t> >& lines)
{
	const int derivnum = len-1;
	const int doublederivnum = len-2;

	float deriv_theta[derivnum];
	float deriv_radius[derivnum];

	float second_deriv_theta[doublederivnum];
	float second_deriv_radius[doublederivnum];
	
	float abs_distance_second_deriv[doublederivnum];

	takeDerivative(theta, radius, deriv_theta, deriv_radius, len);
	takeDerivative(deriv_theta, deriv_radius, second_deriv_theta, second_deriv_radius, derivnum);

	for(int i = 0; i < doublederivnum; i++)
	{
		abs_distance_second_deriv[i] = abs(second_deriv_radius[i]);
	}

	bool linear_map[doublederivnum];
	memset(linear_map, 0, doublederivnum*sizeof(bool));

	int start, stop;
	start = stop = -1;
	for(int i = 0; i < doublederivnum; i++)
	{

		if(abs_distance_second_deriv[i] < zero_tol)
		{
			linear_map[i] = true;
		}

		//start a range
		if( (linear_map[i]) && (start == -1))
		{
			start = i;
		}

		//stop a range
		if( ( !(linear_map[i]) ) && (start != -1) && (i != (doublederivnum-1)) )//we are not at last element, have an open range. set the end to the next element
		{
			stop = i+1;
			lines.push_back( boost::tuple<size_t,size_t>(start, stop) );
			start = stop = -1;
		}
		else if( (( !(linear_map[i]) ) || (i == (doublederivnum-1))) && (start != -1) )//we are at last element, and still have an open range. set the end to the last element
		{
			stop = i+2;

			lines.push_back( boost::tuple<size_t,size_t>(start, stop) );
			start = stop = -1;
		}
	}

	return true;
}
*/

bool findLinearRuns(const float* theta, const float* radius, const size_t len, const double zero_tol, std::deque< boost::tuple<size_t,size_t> >& lines)
{
	assert(zero_tol > 0);

	const int derivnum = len;
	const int doublederivnum = len;

	float deriv_radius[derivnum];

	float second_deriv_radius[doublederivnum];
	
	float abs_distance_second_deriv[doublederivnum];

	takeDerivative_center(theta, radius, deriv_radius, len);
	takeDerivative_center(theta, deriv_radius, second_deriv_radius, derivnum);

	for(int i = 0; i < doublederivnum; i++)
	{
		abs_distance_second_deriv[i] = fabsf(second_deriv_radius[i]);
	}

	bool linear_map[doublederivnum];
	memset(linear_map, 0, doublederivnum*sizeof(bool));

	for(int i = 0; i < doublederivnum; i++)
	{
		if(abs_distance_second_deriv[i] < zero_tol)
		{
			linear_map[i] = true;
		}
	}

	bool inrun = false;
	size_t start = 0;
	size_t stop = 0;
	for(int i = 0; i < doublederivnum; i++)
	{

		if((inrun) && (linear_map[i] == true))
		{
			continue;
		}

		if((inrun) && (linear_map[i] == false))
		{
			stop = i - 1;
			inrun = false;

			boost::tuple<size_t, size_t> run(start, stop);
			lines.push_back(run);
			continue;
		}
		
		if((linear_map[i] == true) && (!inrun))
		{
			start = i;
			inrun = true;
			continue;
		}
	}

	return true;
}


void getLongestRun(const float* x, const float* y, const std::deque< boost::tuple<size_t,size_t> >& lines, boost::tuple<size_t,size_t>& longest)
{
	//fix this to use the lines to index into coord and calculate the length of a line segment. remeber to transform out of polar coord
	float maxdist = -1;
	size_t idx = -1;

	for(size_t i = 0; i < lines.size(); i++)
	{
		size_t idx0 = lines[i].get<0>();		
		size_t idx1 = lines[i].get<1>();

		float dx = x[idx1] - x[idx0];
		float dy = y[idx1] - y[idx0];

		float len = sqrt(dx*dx + dy*dy);

		if(maxdist == len)
		{
			float old_dist_straight = abs(M_PI / double(2) - (lines[idx].get<1>() - lines[idx].get<0>()) / (float(2)) );
			float new_dist_straight = abs(M_PI / double(2) - (lines[i].get<1>() - lines[i].get<0>()) / (float(2)) );

			if(old_dist_straight > new_dist_straight)
			{
				maxdist = len;
				idx = i;
			}
			continue;
		}

		if(maxdist < len)
		{
			maxdist = len;
			idx = i;
		}
	}

	longest = lines[idx];
}

//since output is linear scan, the adjacent points are the closest points
//remove points that have no neighbor with maxdist
void removeIsolatedPoints(const float* x_in, const float* y_in, size_t len_in, float* x_out, float* y_out, size_t& len_out, const float maxdist)
{
	len_out = 0;

	float maxdistsq = maxdist*maxdist;

	//fwd len becomes bwd len, so cache them
	float distsq_fwd, distsq_bwd, n0distsq;
	{
		//unroll first itr	
		float dxf = x_in[0] - x_in[len_in-1];
		float dyf = y_in[0] - y_in[len_in-1];
		n0distsq = distsq_bwd = dxf*dxf + dyf*dyf;
	}
	for(size_t i = 0; i < len_in-1; i++)
	{
		float dxf = x_in[i+1] - x_in[i];
		float dyf = y_in[i+1] - y_in[i];
		distsq_fwd = dxf*dxf + dyf*dyf;

		if( (distsq_fwd < maxdistsq) || (distsq_bwd < maxdistsq) )
		{
			x_out[len_out] = x_in[i];
			y_out[len_out] = y_in[i];
			len_out++;
		}

		distsq_bwd = distsq_fwd;
	}

	//do the last itr using the cached distance
	{
		float dxb = x_in[len_in-1] - x_in[len_in-2];
		float dyb = y_in[len_in-1] - y_in[len_in-2];
		distsq_bwd = dxb*dxb + dyb*dyb;
		if( (n0distsq < maxdistsq) || (distsq_bwd < maxdistsq) )
		{
			x_out[len_out] = x_in[len_in-1];
			y_out[len_out] = y_in[len_in-1];
			len_out++;

		}
	}
}

	bool isRadialClear(const float theta, const float radius, const double angle_tol, const float* t_pt, const float* r_pt, const size_t numpts)
	{
		const float theta_low = theta - angle_tol;
		const float theta_high = theta + angle_tol;

		// count the points within the provided cone
		size_t pts_in_cone = 0;
		for(size_t i = 0; i < numpts; i++)
		{
			const float& t_i = t_pt[i];
			const float& r_i = r_pt[i];

			if((theta_low <= t_i) && (t_i <= theta_high))
			{
				if(r_i <= radius)
				{
					pts_in_cone++;
				}
			}
		}

		//need to allow for noise. this is a stupid way to do so.
		const static size_t pt_thresh = 0;
		if(pts_in_cone > pt_thresh)
		{
			return false;
		}

		return true;
	}

	bool isPathClear(const float theta, const float width, const float distance, const float* t_pt, const float* r_pt, const size_t numpts)
	{
		// count the points within the provided cone
		size_t pts_in_width = 0;

		for(size_t i = 0; i < numpts; i++)
		{
			float rt_pt, rr_pt;
			polar_rotate(t_pt[i], r_pt[i], theta, rt_pt, rr_pt);

			float x, y;
			NAV200::polar2cart(rt_pt, rr_pt, x, y);

			if( (abs(x) < (width/2.0)) && (y <= distance))
			{
				pts_in_width++;
			}
		}
		std::cout << pts_in_width << std::endl;
		//need to allow for noise. this is a stupid way to do so.
		const static size_t pt_thresh = 3;
		if(pts_in_width > pt_thresh)
		{
			return false;
		}

		return true;
	}

	//lines must be in order such that successive lines are the closest match
	bool collectNearRuns(const float* theta, const float* radius, const size_t len, const double distance, std::deque< boost::tuple<size_t,size_t> >& lines, std::deque< boost::tuple<size_t,size_t> >& grown_lines)
{
	bool started = false;
	size_t segment_start = 0;
	size_t segment_end = 0;
	for(size_t i = 0; i < (lines.size() - 1); i++)
	{
		const boost::tuple<size_t,size_t>& pt = lines[i];
		const float& startt = theta[pt.get<0>()];
		const float& startr = radius[pt.get<0>()];
		const float& stopt = theta[pt.get<1>()];
		const float& stopr = radius[pt.get<1>()];

		const boost::tuple<size_t,size_t>& next_pt = lines[i+1];
		const float& next_startt = theta[next_pt.get<0>()];
		const float& next_startr = radius[next_pt.get<0>()];
		const float& next_stopt = theta[next_pt.get<1>()];
		const float& next_stopr = radius[next_pt.get<1>()];

		float d = polar_distance(stopt, stopr, next_stopt, next_stopr);

		if(d < distance)
		{
			if(!started)
			{
				started = true;
				segment_start = i;
			}
		}
		else
		{
			if(started)
			{
				started = false;
				segment_end = i;

				boost::tuple<size_t,size_t> seg;
				seg.get<0>() = segment_start;
				seg.get<1>() = segment_end;
				grown_lines.push_back(seg);
			}
		}
	}
	return false;
}

	void fitPtsToLine(const float* theta, const float* radius, size_t len)
	{
		std::list< std::vector<cv::Point2f> > divided_pts;

		//need to perform initial subdivision based on slope distance or something

		//do a linear fit
		//cv::Vec6f line;
		//cv::fitLine(divided_pts, line, CV_DIST_L2, 0, .01, .01);
	}

}

