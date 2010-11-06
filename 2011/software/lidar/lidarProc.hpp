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
			destX[i] = srcX[i] + (srcX[i+1] - srcX[i]) / (T(2));
			destY[i] = (srcY[i+1] - srcY[i]) / (srcX[i+1] - srcX[i]);
		}
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

	bool findLinearRuns(const float* theta, const float* radius, const size_t len, const double zero_tol, std::deque< boost::tuple<size_t,size_t> >& lines);

	void getLongestRun(const float* theta, const float* radius, const std::deque< boost::tuple<size_t,size_t> >& lines, boost::tuple<size_t,size_t>& longest);

	void removeIsolatedPoints(const float* x_in, const float* y_in, size_t len_in, float* x_out, float* y_out, size_t& len_out, const float dist);

	void deepClose(cv::Mat& im, size_t itr);
}

#endif //LIDARPROC_CC
