#ifndef GNUPLOT_H
#define GNUPLOT_H 1

#include <cstdio>
#include <sstream>
#include <string>

/*#include <flens/crs.h> 
#include <flens/densevector.h>
#include <flens/fullstorage.h>
#include <flens/generalmatrix.h>
#include <flens/sparsematrix.h>
#include <flens/storage.h>*/

#include <flens/flens.h>

namespace gnuplot {

#ifndef MACOS
#    define DEFAULT_TERMINAL "x11"
#    define GNUPLOT_CMD "gnuplot -persist"
#else
#    define DEFAULT_TERMINAL "aqua"
#    define GNUPLOT_CMD "/usr/local/bin/gnuplot -persist"
#endif

using namespace flens;

class Gnuplot
{
    public:
        Gnuplot();
        
        void
        plot(const DenseVector<Array<double> > &y);

        void
        plot(const DenseVector<Array<double> > &x, 
             const DenseVector<Array<double> > &y);

        void
        plot(const DenseVector<Array<double> > &x, 
             const DenseVector<Array<double> > &y, 
             const DenseVector<Array<double> > &z);
             
        void
        execute(const std::string &command);

        void
        writeData(const double *x, int length, int strideX = 1);
        
        void
        writeData(const double *x, const double *y, int length, 
                  int strideX = 1, int strideY = 1);
        
        void
        writeData(const double *x, const double *y, const double *z, 
                 int length, int strideX = 1, int strideY = 1, int strideZ = 1);
                   
        void
        setTerminal(const std::string &terminal = DEFAULT_TERMINAL);
        
        void
        setOutput(const std::string &filename);

        template <typename T>
            void
            spy(const SparseSymmetricMatrix<CRS<T> > &A, 
                double tolerance=1e-15);
                
        void
        holdOn();
        
        void
        holdOff();
            
        std::stringstream options;
        std::stringstream ranges;
        std::stringstream set;
                
    private:
        // not allowed!
        Gnuplot(const Gnuplot &rhs);

        // not allowed!        
        Gnuplot &
        operator=(const Gnuplot &rhs);

        void
        _initialize();
        
        void
        _flush();

        FILE *_fd;
        
        
        std::stringstream _command;
        std::stringstream _data;
        bool _holdOn;
		bool _output;
		std::string _outputFile;
};

} // namespace gnuplot

#include "Gnuplot.tcc"

#endif // GNUPLOT_H
