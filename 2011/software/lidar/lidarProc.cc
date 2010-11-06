#include "NAV200.hpp"

#include "lidarProc.hpp"

#include "boost/foreach.hpp"



namespace lidarProc
{

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

//just like cv::morphologyEx(morphLidar, morphLidar, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), itr);
void deepClose(cv::Mat& im, size_t itr)
{
	for(size_t i = 0; i < itr; i++)
	{
		cv::dilate(im, im, cv::Mat());
	}

	for(size_t i = 0; i < itr; i++)
	{
		cv::erode(im, im, cv::Mat());
	}
}
}

