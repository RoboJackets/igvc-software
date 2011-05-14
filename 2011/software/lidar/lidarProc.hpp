#ifndef LIDARPROC_CC
#define LIDARPROC_CC

#include "boost/tuple/tuple.hpp"

#include "opencv/cv.h"

namespace lidarProc
{

	//[a,b,c, ...]
	//[b - a, c - b, ...]
	template<typename T>
	static void takeDerivative(const T* srcX, const T* srcY, T* destX, T* destY, const int srclen)
	{
		for(int i = 0; i < (srclen-1); i++)
		{
			//destX[i] = srcX[i] + (srcX[i+1] - srcX[i]) / (T(2));
			destX[i] = srcX[i];
			destY[i] = (srcY[i+1] - srcY[i]) / (srcX[i+1] - srcX[i]);
		}
	}

	template<typename T>
	static void takeDerivative_bkwd(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		for(int i = 0; i < (srclen-1); i++)
		{
			destY[i] = (srcY[i] - srcY[i-1]) / (srcX[i] - srcX[i-1]);
		}
	}

	template<typename T>
	static void takeDerivative_fwd(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		for(int i = 0; i < (srclen-1); i++)
		{
			destY[i] = (srcY[i+1] - srcY[i]) / (srcX[i+1] - srcX[i]);
		}
	}

	//2nd order
	//dest is same len as src
	template<typename T>
	static void takeDerivative_center(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		destY[0] = (srcY[1] - srcY[0]) / (srcX[1] - srcX[0]);
		for(int i = 1; i < (srclen-1); i++)
		{
			destY[i] = (srcY[i+1] - srcY[i-1]) / (srcX[i+1] - srcX[i-1]);
		}
		destY[srclen-1] = (srcY[srclen-1] - srcY[srclen-2]) / (srcX[srclen-1] - srcX[srclen-2]);
	}

	template <typename T>
	void runavg(const T* in, T* out, const size_t len, const size_t n)
	{
		const size_t incn = n+1;

		T sum = 0;
		for(size_t i = 0; i < n; i++)
		{
			sum += in[i];
			out[i] = sum / (i+1);
		}

		//unroll the first op
		sum = 0;
		for(size_t j = 0; j <= n; j++)
		{
			sum += in[n-j];
		}
		out[n] = sum / incn;

		for(size_t i = n+1; i < len; i++)
		{
			sum -= in[i-n-1];
			sum += in[i];
			out[i] = sum / incn;
		}
	}

	template<typename T>
	T cart_distance_sq(const T& x0, const T& y0, const T& x1, const T& y1)
	{
		T dx = x1 - x0;
		T dy = y1 - y0;
		return dx*dx+dy*dy;
	}

	template<typename T>
	T polar_distance_sq(const T& t0, const T& r0, const T& t1, const T& r1)
	{
		T dt;
		if(t1 > t0)
		{
			dt = t1 - t0;
		}
		else
		{
			dt = t0 - t1;
		}

		return r0*r0 + r1*r1 - 2*r0*r1*cos(dt);
	}

	template<typename T>
	T cart_distance(const T& x0, const T& y0, const T& x1, const T& y1)
	{
		return sqrt(cart_distance_sq(x0, y0, x1, y1));
	}

	template<typename T>
	T polar_distance(const T& t0, const T& r0, const T& t1, const T& r1)
	{
		return sqrt(polar_distance_sq(t0, r0, t1, r1));
	}

	inline float cart_rotate(const float& x, const float& y, const float& theta, float& rx, float& ry)
	{
		float st;
		float ct;
		sincosf(theta, &st, &ct);

		rx = ct * x - st*y;
		ry = st * x + ct*y;
	}

	inline float polar_rotate(const float& t, const float& r, const float& theta, float& rt, float& rr)
	{
		rt = t + theta;
		rr = r;
	}


	bool findLinearRuns(const float* theta, const float* radius, const size_t len, const double zero_tol, std::deque< boost::tuple<size_t,size_t> >& lines);

	bool collectNearRuns(const float* theta, const float* radius, const size_t len, const double distance, std::deque< boost::tuple<size_t,size_t> >& lines, std::deque< boost::tuple<size_t,size_t> >& grown_lines);

	void getLongestRun(const float* theta, const float* radius, const std::deque< boost::tuple<size_t,size_t> >& lines, boost::tuple<size_t,size_t>& longest);

	void removeIsolatedPoints(const float* x_in, const float* y_in, size_t len_in, float* x_out, float* y_out, size_t& len_out, const float dist);

	bool isRadialClear(const float theta, const float radius, const double angle_tol, const float* t_pt, const float* r_pt, const size_t numpts);
	bool isPathClear(const float theta, const float width, const float* t_pt, const float* r_pt, const size_t numpts);
}

#endif //LIDARPROC_CC
