//1st derivatives
	//1st order accurate
	template<typename T>
	static void takeDerivative_bkwd(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		for(int i = 0; i < (srclen-1); i++)
		{
			destY[i] = (srcY[i] - srcY[i-1]) / (srcX[i] - srcX[i-1]);
		}
	}

	//1st order accurate
	template<typename T>
	static void takeDerivative_fwd(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		for(int i = 0; i < (srclen-1); i++)
		{
			destY[i] = (srcY[i+1] - srcY[i]) / (srcX[i+1] - srcX[i]);
		}
	}

	//2nd order, with 1st order endpts
	//dest is same len as src
	template<typename T>
	static void takeDerivative_center(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		destY[0] = (srcY[1] - srcY[0]) / (srcX[1] - srcX[0]);
		for(int i = 1; i < (srclen-1); i++)
		{
			T approx_dx = (srcX[i+1] - srcX[i-1]) / T(2);
			destY[i] = T(1.0/2.0) * (srcY[i+1] - srcY[i-1]) / (approx_dx);
		}
		destY[srclen-1] = (srcY[srclen-1] - srcY[srclen-2]) / (srcX[srclen-1] - srcX[srclen-2]);
	}

//2nd derivatives
	//1st order
	template<typename T>
	static void take2ndDerivative_fwd(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		for(int i = 1; i < (srclen-1); i++)
		{
			T approx_dx = abs(srcX[i+2] - srcX[i]) / T(3);
			destY[i] = (srcY[i] - T(2) * srcY[i+1] + srcY[i+2]) / (approx_dx * approx_dx);
		}
	}

	//2nd order
	//dest is same len as src
	template<typename T>
	static void take2ndDerivative_center(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		{
		T approx_dx = (srcX[2] + srcX[1] + srcX[0]) / T(3);
		destY[0] = (srcY[0] - T(2) * srcY[1] + srcY[2]) / (approx_dx * approx_dx);
		}
		
		for(int i = 1; i < (srclen-1); i++)
		{
			T approx_dx = abs(srcX[i+1] - srcX[i-1]) / T(3);
			destY[i] = (srcY[i+1] - T(2) * srcY[i] + srcY[i-1]) / (approx_dx*approx_dx);
		}

		{
		T approx_dx = (srcX[srclen-3] + srcX[srclen-2] + srcX[srclen-1]) / T(3);
		destY[srclen-1] = (srcY[srclen-1] - T(2) * srcY[srclen-2] + srcY[srclen-3]) / (approx_dx * approx_dx);
		}
	}

	//1st order
	template<typename T>
	static void take2ndDerivative_bwd(const T* srcX, const T* srcY, T* destY, const int srclen)
	{
		for(int i = 1; i < (srclen-1); i++)
		{
			T approx_dx = abs(srcX[i-2] + srcX[i]) / T(3);
			destY[i] = (srcY[i] - T(2) * srcY[i-1] + srcY[i-2]) / (approx_dx * approx_dx);
		}
	}
