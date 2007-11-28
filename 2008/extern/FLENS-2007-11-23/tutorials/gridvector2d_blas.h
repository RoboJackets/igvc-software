#ifndef TUTORIALS_GRIDVECTOR2D_BLAS_H
#define TUTORIALS_GRIDVECTOR2D_BLAS_H 1

namespace flens {

// x'*y
double
dot(const GridVector2D &x, const GridVector2D &y)
{
    const GridVector2D::Grid &X = x.grid;
    const GridVector2D::Grid &Y = y.grid;
    
    assert(X.numRows()==Y.numRows());
    assert(X.numCols()==Y.numCols());
        
    int m = Y.lastRow()-1;
    int n = Y.lastCol()-1;

    double d=0;
    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            d += X(i,j)*Y(i,j);
        }
    }
    return d;
}

// y = x
void
copy(const GridVector2D &x, GridVector2D &y)
{
    y.grid = x.grid;
}

// x = alpha*x
void
scal(double alpha, GridVector2D &x)
{
    x.grid *= alpha;
}

// y = alpha*x
void
axpy(double alpha, const GridVector2D &x, GridVector2D &y)
{
    y.grid += alpha*x.grid;
}

// y = alpha*A*x + beta*y
void
mv(double alpha, const Poisson2D &A, const GridVector2D &x,
   double beta, GridVector2D &y)
{
    assert(&x!=&y);

    const GridVector2D::Grid &X = x.grid;
    GridVector2D::Grid       &Y = y.grid;

    if ((Y.numRows()!=X.numRows()) || (Y.numCols()!=X.numCols())) {
        Y.resize(X.numRows(), X.numCols(), X.firstRow(), X.firstCol());
    }

    int m = Y.lastRow()-1;
    int n = Y.lastCol()-1;

    alpha *= A.rh*A.rh;
    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            Y(i,j)  = beta*Y(i,j);
            Y(i,j) += alpha*(4*X(i,j)-X(i-1,j)-X(i+1,j)-X(i,j-1)-X(i,j+1));
        }
    }
}

// output in a format gnuplot understands
std::ofstream &
operator<<(std::ofstream &out, const GridVector2D &x)
{
    const GridVector2D::Grid &X = x.grid;
    
    out.precision(6);
    out.setf(std::ios::fixed);
    double h = 1./x.rh;
    for (int i=X.firstRow(); i<=X.lastRow(); ++i) {
        for (int j=X.firstCol(); j<=X.lastCol(); ++j) {
            double x = j*h;
            double y = i*h;
            out.width(12);
            out << x << " " << y << " " << X(i,j) << std::endl;
        }
        out << std::endl;
    }
    return out;
}

} // namespace flens

#endif // TUTORIALS_GRIDVECTOR2D_BLAS_H
