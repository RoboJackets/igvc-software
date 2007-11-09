#include <cmath>

namespace gnuplot {

template <typename T>
void
Gnuplot::spy(const SparseSymmetricMatrix<CRS<T> > &A, double tolerance)
{
	_command << "unset xtics; unset ytics; " << std::endl;
    _command << "plot "
                "[ 0 : " << A.numCols()+1 << " ] " 
                "[ 0 : " << A.numRows()+1 << " ] "
                " '-' notitle ";
    if (A.numRows()>=1024) {
	    _command << "with dots ";
	} else {
		_command << "with points ";
	}
	_command << options.str() << std::endl;
 	
	typename SparseSymmetricMatrix<CRS<T> >::const_iterator it = A.begin();
	for (; it!=A.end(); ++it) {
		if (std::abs(it->second) >= tolerance) {
			_command << it->first.second << " " << A.numRows()-it->first.first+1 << std::endl;
			_command << it->first.first  << " " << A.numRows()-it->first.second+1 << std::endl;
		}
    }
    _command << "e" << std::endl;
    options.str("");
    _flush();
}

} // namespace gnuplot
