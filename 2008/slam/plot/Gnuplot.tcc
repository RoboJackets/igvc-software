#include <cassert>
#include <sys/wait.h>

Gnuplot::Gnuplot(void) {
	Hold = OFF;
	gnuplotFD = popen(GNUPLOT_CMD,"w");
	assert(gnuplotFD);
	plotCommand.str("");
	setCommand.str("");
	setCommand << GNUPLOT_DEFAULT_SET << endl;
}

//////////////////////
// #ifdef FLENS
template<typename T>
void Gnuplot::plot(const DenseVector<Array<T> > &y) {
	plot<T>(y.length(), y.data());
	return;
}

template<typename T>
void Gnuplot::plot(const DenseVector<Array<T> > &x, const DenseVector<Array<T> > &y) {
	assert(x.length() == y.length());
	plot<T>(x.length(), x.data(), y.data());
	return;
}

template<typename T>
void Gnuplot::plot(const DenseVector<Array<T> > &x, const DenseVector<Array<T> > &y, const DenseVector<Array<T> > &z) {
	assert(x.length() == y.length());
	assert(x.length() == z.length());
	mesh<T>(x.length(), x.data(), y.data(), z.data());
	return;
}
//////////////////////

template<typename T>
void Gnuplot::plot(int length, const T *x, const T *y) {
	if( Hold == ON ) {
		if(subplotingOn) {
			; // set the orgin and size
		}
	} else {
		plotCommand.str("");
		setCommand << "reset;" << GNUPLOT_DEFAULT_SET << endl;
		if(subplotingOn) {
			//setCommand << << endl; // redo the set orgin and set size
		}
	}
	plotCommand << "plot '-' w linespoints" << endl; // << ranges << options;

	assert(length > 0);
	assert(x);

	if( y ) {
		for(int i = 0; i < length; i++) {
			plotCommand << x[i] << "\t" << y[i] << endl;
		}
	} else {
		for(int i = 0; i < length; i++) {
			plotCommand << x[i] << endl;
		}
	}
	plotCommand << "e" << endl;
	//cout << plotCommand.str() << endl;
	execute(setCommand);
	setCommand.str("");
	execute(plotCommand);

	return;
}

template<typename T>
void Gnuplot::mesh(int length, const T *x, const T *y, const T *z) {
	if( Hold == ON ) {
		if(subplotingOn) {
			; // set the orgin and size
		}
	} else {
		plotCommand.str("");
		setCommand << "reset;" << GNUPLOT_DEFAULT_SET << endl;
		if(subplotingOn) {
			//setCommand << << endl; // redo the set orgin and set size
		}
	}
	plotCommand << "splot '-' w linespoints" << endl; // << ranges << options;

	assert(length > 0);
	assert(x);
	assert(y);
	assert(z);

	for(int i = 0; i < length; i++) {
		plotCommand << x[i] << "\t" << y[i] << "\t" << z[i] << endl;
	}

	plotCommand << "e" << endl;
	//cout << plotCommand.str() << endl;
	execute(setCommand);
	setCommand.str("");
	execute(plotCommand);

	return;
}

void Gnuplot::axis(double xMin, double xMax) {
	setCommand << "set xrange [" << xMin << ":" << xMax << "]; clear;" << endl;
	execute(setCommand);
	setCommand.str("");
	execute(plotCommand);
	return;
}

void Gnuplot::axis(double xMin, double xMax, double yMin, double yMax) {
	setCommand << "set xrange [" << xMin << ":" << xMax << "];";
	setCommand << "set yrange [" << yMin << ":" << yMax << "]; clear;" << endl;
	execute(setCommand);
	setCommand.str("");
	execute(plotCommand);
	return;
}

void Gnuplot::axis(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax) {
	setCommand << "set xrange [" << xMin << ":" << xMax << "];";
	setCommand << "set yrange [" << yMin << ":" << yMax << "];";
	setCommand << "set zrange [" << zMin << ":" << zMax << "]; clear;" << endl;
	execute(setCommand);
	setCommand.str("");
	execute(plotCommand);
	return;
}

void Gnuplot::title(string title) {
	setCommand << "set title '" << title << "'; clear;" << endl;
	execute(setCommand);
	setCommand.str("");
	//if(plotCommand.str() != "") {
		execute(plotCommand);
	/*} else {
		plot [][0:1] 2;
	}*/
	return;
}

void Gnuplot::xlabel(string label) {
	setCommand << "set xlabel '" << label << "'; clear;" << endl;
	execute(setCommand);
	setCommand.str("");
	execute(plotCommand);
	return;
}

void Gnuplot::ylabel(string label) {
	setCommand << "set ylabel '" << label << "'; clear;" << endl;
	execute(setCommand);
	setCommand.str("");
	execute(plotCommand);
	return;
}

void Gnuplot::zlabel(string label) {
	setCommand << "set zlabel '" << label << "'; clear;" << endl;
	execute(setCommand);
	setCommand.str("");
	execute(plotCommand);
	return;
}
#if 0
void subplot(int numRows, int numCols, int index) {
	if(subplotingOn) {
		setCommand << "set multiplot;" << endl;
		subplotingOn = true;
	}

	origin = ;
	size = ;

}
#endif
void Gnuplot::hold(HoldType value) {
	if( (value == ON) && (Hold = OFF) ) {
		if(!subplotingOn) {
			setCommand << "set multiplot;" << endl;
		}
	} else if ( (value == OFF) && (Hold = ON) ) {
		if(!subplotingOn) {
			setCommand << "unset multiplot;" << endl;
		}
	}

	execute(setCommand);
	setCommand.str("");
	Hold = value;
	return;
}

void Gnuplot::execute(const stringstream &command) {
	assert(gnuplotFD);
	fprintf(gnuplotFD, "%s\n", command.str().c_str()); 
	fflush(gnuplotFD);
	return;
}

