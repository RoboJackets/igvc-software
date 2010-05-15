
#include "boost/tuple/tuple.hpp"
#include "boost/foreach.hpp"
#include <list>
#include <cmath>
#include <iostream>

typedef boost::tuple<int,int> range;

template<typename T>
void ref_abs(T& n)
{
	n = abs(n);
}

template<typename T>
void apply_func(std::list<T>& v, void (*f)(T&))
{
	BOOST_FOREACH(T& e, v)
	{
		f(e);
	}
}

template<typename T>
void apply_func(T* v, size_t len, void (*f)(T&))
{
	for(size_t i = 0; i < len; i++)
	{
		f(v[i]);
	}
}

//[a,b,c, ...]
//[b - a, c - b, ...]
void takeDerivative(double* src, double* dest, const double srclen)
{
	for(int i = 0; i < (srclen-1); i++)
	{
		dest[i] = src[i+1] - src[i];
	}
}

template<typename T>
void printarray(T* v, size_t len)
{
	std::cout << "<" << v[0] << ", ";
	for(int i = 1; i < (len-1); i++)
	{
		std::cout << v[i] << ", ";
	}
	std::cout << v[len-1] << ">\n";
}

template<typename T>
void printarray(T* v, size_t start, size_t stop)
{
	std::cout << "<" << v[start] << ", ";
	for(int i = start+1; i < stop; i++)
	{
		std::cout << v[i] << ", ";
	}
	std::cout << v[stop] << ">\n";
}

void findRamp()
{
	const double zero_tol = .01;

	//double ranges[] = {1,2,3,4,5,0,2,4,6,8,0,3,6,9,0,4,9,16,25, -2,-4,-6, .1, .3, .5, 5, 15, 25};
	//double ranges[] = {0, 1, 8, 27, 4, 5, 6, 9, 10};// this works
	double ranges[] = {0, 1, 8, 27, 4, 5, 6, 9};// this does not
	//double ranges[] = {1,2,3,4,5};

	const size_t sampnum = sizeof(ranges) / sizeof(double);
	const size_t derivnum = sampnum-1;
	const size_t doublederivum = derivnum-1;
	
	double deriv[derivnum];
	double doublederiv[doublederivum];
	double abs_doublederiv[doublederivum];

	bool linear_map[sampnum] = {false};
	int rampmap[sampnum] = {false};//differ between linear runs with different slopes

	takeDerivative(ranges, deriv, sampnum);
	takeDerivative(deriv, doublederiv, derivnum);

	memcpy(abs_doublederiv, doublederiv, doublederivum*sizeof(double));
	apply_func(abs_doublederiv, doublederivum, ref_abs);

	for(int i = 0; i < doublederivum; i++)
	{
		if(abs_doublederiv[i] < zero_tol)
		{
			linear_map[i] = true;
		}
	}
	
	std::list< boost::tuple<int,int> > lines;
	int start, stop;
	start = stop = -1;
	for(int i = 0; i < doublederivum; i++)
	{
		//start a range
		if( (abs_doublederiv[i] < zero_tol) && (start == -1))
		{
			start = i;
		}

		//stop a range
		if( ((abs_doublederiv[i] >= zero_tol)) && (start != -1) && (i != (doublederivum-1)) )
		{
			stop = i+1;
			lines.push_back( boost::tuple<size_t,size_t>(start, stop) );
			start = stop = -1;
		}
		else if( ((abs_doublederiv[i] >= zero_tol) || (i == (doublederivum-1))) && (start != -1) )//we are at last element, and still have an open range. set the end to the last element
		{
			stop = i+2;
			lines.push_back( boost::tuple<size_t,size_t>(start, stop) );
			start = stop = -1;
		}
	}

	std::cout << "range:\t\t\t";printarray(ranges, sampnum);
	std::cout << "deriv:\t\t\t";printarray(deriv, derivnum);
	std::cout << "doublederiv:\t\t";printarray(doublederiv, doublederivum);
	std::cout << "abs_doublederiv:\t";printarray(abs_doublederiv, doublederivum);

	BOOST_FOREACH(range& e, lines)
	{
		std::cout << "start: " << e.get<0>() << ", stop: " << e.get<1>() << std::endl;
		std::cout << "\t";
		printarray(ranges, e.get<0>(), e.get<1>());
	}

}

int main()
{
	findRamp();
	return 0;
}
