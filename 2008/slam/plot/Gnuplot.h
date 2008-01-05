#ifndef GNUPLOT_H
#define GNUPLOT_H 1

#include <cstdio>
#include <sstream>
#include <string>

//make this work with or without flens
#include <flens/flens.h>

#define GNUPLOT_CMD			"gnuplot -persist"
#define GNUPLOT_DEFAULT_SET	"set mouse; unset key;"

using namespace std;
using namespace flens;

enum HoldType {
	OFF,
	ON
	//ALL
	//SOME
};

class PlotData;

class Gnuplot {
public:
	Gnuplot();



	template<typename T>
	void plot(const DenseVector<Array<T> > &y);

	template<typename T>
	void plot(const DenseVector<Array<T> > &x, const DenseVector<Array<T> > &y);

	template<typename T>
	void plot(const DenseVector<Array<T> > &x, const DenseVector<Array<T> > &y, const DenseVector<Array<T> > &z);



	template<typename T>
	void plot(int length, const T *x, const T *y = 0);

	template<typename T>
	void mesh(int length, const T *x, const T *y, const T *z);

	//void replot(const DenseVector<Array<double> > &y); // appends stuff to the last plot

	void axis(double xMin, double xMax);
	void axis(double xMin, double xMax, double yMin, double yMax);
	void axis(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);
	void title(string title);
	void xlabel(string label);
	void ylabel(string label);
	void zlabel(string label);

	//void gset();
	//void gshow();

	void hold(HoldType value);
                
private:
	/* Prevent instantiation */
	Gnuplot(const Gnuplot &rhs);

	/* Prevent assignment */       
	Gnuplot & operator=(const Gnuplot &rhs);

	void execute(const stringstream &command);

	FILE *gnuplotFD;

	//origin;
	//size;

	stringstream setCommand;
	stringstream plotCommand;
	HoldType Hold;
	bool subplotingOn;
};

#include "Gnuplot.tcc"

#endif // GNUPLOT_H
