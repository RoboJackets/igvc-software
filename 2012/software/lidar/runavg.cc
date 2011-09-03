#include <cstdlib>
#include <iostream>

//average the last n values with the current value
//operates on (n+1) pts at a time
//first (n-1) points are the average of all points up to that point
template <typename T>
void runavg(const T* in, T* out, const size_t len, const size_t n)
{
	const size_t incn = n+1;

	T sum = 0;
	for(int i = 0; i < n; i++)
	{
		sum += in[i];
		out[i] = sum / (i+1);
	}

	//unroll the first op
	sum = 0;
	for(int j = 0; j <= n; j++)
	{
		sum += in[n-j];
	}
	out[n] = sum / incn;

	for(int i = n+1; i < len; i++)
	{
		sum -= in[i-n-1];
		sum += in[i];
		out[i] = sum / incn;
	}
}

int main()
{
	float in[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	float out[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

	runavg(in, out, 9, 3);

	std::cout << "<";
	for(int i = 0; i < 8; i++)
	{
		std::cout << out[i] << ", ";
	}
	std::cout << out [8] << ">\n";
}
