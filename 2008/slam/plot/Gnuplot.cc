#include <cassert>
#include <sys/wait.h>
#include "Gnuplot.h"

namespace gnuplot {

using namespace flens;

Gnuplot::Gnuplot(void)
    : _holdOn(false)
{
    _fd = popen(GNUPLOT_CMD,"w");
    _initialize();
}

void
Gnuplot::plot(const flens::DenseVector<Array<double> > &y)
{
    assert(_fd);
   // DenseVector<Array<double> > x = _(1, y.length());
    
    if (_command.str()=="") {
        _command << " plot " << ranges.str();
    } else {
        _command << ", ";
    }
    _command << " '-' "; 
    if (options.str().find("$0",0)==std::string::npos) {
        _command << " u ($0+1):1 ";
    }
    _command << options.str();
    options.str(""); ranges.str("");
    writeData(y.data(), y.length(), y.stride());
    _flush();
}

void
Gnuplot::plot(const flens::DenseVector<Array<double> > &x,
              const flens::DenseVector<Array<double> > &y)
{
    assert(_fd);
    assert(x.length()==y.length());
    if (_command.str()=="") {
        _command << " plot " << ranges.str();
    } else {
        _command << ", ";
    }
    _command << " '-' " << options.str();
    options.str(""); ranges.str("");
    writeData(x.data(), y.data(), x.length(), x.stride(), y.stride());
    _flush();
}

void
Gnuplot::plot(const flens::DenseVector<Array<double> > &x,
              const flens::DenseVector<Array<double> > &y,
              const flens::DenseVector<Array<double> > &z)
{
    assert(_fd);
    assert(x.length()==y.length());
    assert(x.length()==z.length());
    if (_command.str()=="") {
        _command << "splot " << ranges.str();
    } else {
       
         _command.str().substr(0,5)="splot";
        _command << ", ";
    }
    _command << " '-' " << options.str();
    options.str(""); ranges.str("");
    writeData(x.data(), y.data(), z.data(), x.length(), x.stride(), y.stride(), 
              z.stride());
    _flush();
}

void
Gnuplot::execute(const std::string &command)
{
    if (_holdOn){
        if( (_command.str().substr(0,4)=="plot") 
                  && (command.substr(0,4)=="plot")) {
            _command << ", " << command.substr(4,command.length()-1) 
                     << " " << options.str();        
        }
        if( (_command.str().substr(0,5)=="splot") 
                  && (command.substr(0,5)=="splot")) {
            _command << ", " << command.substr(5,command.length()-1) 
                     << " " << options.str();
        }
    } else {
        _command.str() = "";
        _command << command << " " << options.str();
    }
    options.str("");
    _flush();
}

void
Gnuplot::writeData(const double *x, int length, int strideX)
{
    assert(_fd);
    for (int i=0; i<length; i++) {
        _data << *x << std::endl;
        x += strideX;
    }
    _data << "e" << std::endl;
}

void
Gnuplot::writeData(const double *x, const double *y, int length, 
                   int strideX, int strideY)
{
    assert(_fd);
    for (int i=0; i<length; i++) {
        _data << *x << " " << *y << std::endl;
        x += strideX;
        y += strideY;
    }
    _data << "e" << std::endl;
}

void
Gnuplot::writeData(const double *x, const double *y, const double *z, 
                   int length, int strideX, int strideY, int strideZ)
{
    assert(_fd);
    for (int i=0; i<length; i++) {
        _data << *x << " " << *y << " " <<  *z << std::endl;
        x += strideX;
        y += strideY;
        z += strideZ;
    }
    _data << "e" << std::endl;
}

void
Gnuplot::setTerminal(const std::string &terminal)
{
    set << "set terminal " << terminal << std::endl;
}

void
Gnuplot::setOutput(const std::string &filename)
{
	set << "set output '" << filename << "'" << std::endl;
	//_command << "set output " << filename << std::endl; // so richtig!
	_output = true;
	_outputFile = filename;
}

void
Gnuplot::holdOn()
{
    _holdOn = true;
}

void
Gnuplot::holdOff()
{
    _command.str("");
	set.str("");
    _data.str("");
    _holdOn = false;
}
//------------------------------------------------------------------------------

void
Gnuplot::_initialize()
{
    set << "set mouse; set terminal " << DEFAULT_TERMINAL << std::endl;
    _flush();
}

void
Gnuplot::_flush()
{
	if(_output){
        int pid =fork();
        if(pid == 0){
            execlp(">", ">",_outputFile.c_str(), (char *) 0);
            exit(0);
        }else{
           wait(&pid);
        }
	}
    
	fprintf(_fd, "%s\n", set.str().c_str());
    fprintf(_fd, "%s\n", _command.str().c_str()); 
    fprintf(_fd, "%s\n", _data.str().c_str()); 
//    std::cout << set.str() << std::endl;
//    std::cout << _command.str() << std::endl; 
//    std::cout << _data.str() << std::endl; 

    if (!_holdOn) {
		set.str("");
        _command.str("");
        _data.str("");
        fprintf(_fd, "%s\n", "reset\n");
    }
    options.str("");
    ranges.str("");
    fflush(_fd);
}

} // namespace gnuplot

