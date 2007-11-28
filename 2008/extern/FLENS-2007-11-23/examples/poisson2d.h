namespace flens {

//-- GaussSeidel ---------------------------------------------------------------

template <typename MA, typename VB>
class GaussSeidel;

template <typename MA, typename VB>
struct TypeInfo<GaussSeidel<MA, VB> >
{
    typedef GaussSeidel<MA, VB> Impl;
    typedef double              ElementType;
};

template <typename MA, typename VB>
class GaussSeidel
    : public SymmetricMatrix<GaussSeidel<MA, VB> >
{
    public:
        typedef VB VectorType;

        GaussSeidel(const MA &_A, const VB &_b, double _omega)
            : A(_A), b(_b), omega(_omega)
        {
        }

        const MA  &A;
        const VB  &b;
        double    omega;
};

template <typename ALPHA, typename MA, typename VB,
          typename VX, typename BETA, typename VY>
void
mv(ALPHA alpha, const GaussSeidel<MA, VB> &A, const Vector<VX> &x,
   BETA beta, Vector<VY> &y)
{
    assert(alpha==ALPHA(1));
    assert(beta==BETA(0));
    assert(&x==&y);
    mv(A, y.impl());
}

//-- GaussSeidel_RedBlack ------------------------------------------------------

template <typename MA, typename VB>
class GaussSeidel_RedBlack;

template <typename MA, typename VB>
struct TypeInfo<GaussSeidel_RedBlack<MA, VB> >
{
    typedef GaussSeidel_RedBlack<MA, VB> Impl;
    typedef double                       ElementType;
};

template <typename MA, typename VB>
class GaussSeidel_RedBlack
    : public SymmetricMatrix<GaussSeidel_RedBlack<MA, VB> >
{
    public:
        typedef VB VectorType;

        GaussSeidel_RedBlack(const MA &_A, const VB &_b, double _omega)
            : A(_A), b(_b), omega(_omega)
        {
        }

        const MA  &A;
        const VB  &b;
        double    omega;
};

template <typename ALPHA, typename MA, typename VB,
          typename VX, typename BETA, typename VY>
void
mv(ALPHA alpha, const GaussSeidel_RedBlack<MA, VB> &A, const Vector<VX> &x,
   BETA beta, Vector<VY> &y)
{
    assert(alpha==ALPHA(1));
    assert(beta==BETA(0));
    assert(&x==&y);
    mv(A, y.impl());
}

//-- GridVector2D --------------------------------------------------------------

class GridVector2D;

template <>
struct TypeInfo<GridVector2D>
{
    typedef GridVector2D Impl;
    typedef double       ElementType;
};

class GridVector2D
    : public Vector<GridVector2D>
{
    public:
        typedef GeMatrix<FullStorage<double, RowMajor> >  Grid;

        GridVector2D()
            : rh(0)
        {
        }


        GridVector2D(int _rh)
            : rh(_rh), grid(_(0,rh),_(0,rh))
        {
        }

        GridVector2D &
        operator=(double value)
        {
            grid = value;
            return *this;
        }

        template <typename RHS>
        GridVector2D &
        operator=(const Vector<RHS> &rhs)
        {
            copy(rhs.impl(), *this);
            return *this;
        }

        template <typename RHS>
        GridVector2D &
        operator+=(const Vector<RHS> &rhs)
        {
            axpy(1, rhs.impl(), *this);
            return *this;
        }

        int    rh;
        Grid   grid;
};

double
norm(const GridVector2D &x)
{
    const GridVector2D::Grid &X = x.grid;
    double h = 1./x.rh;

    double r = 0;
    for (int i=X.firstRow(); i<=X.lastRow(); ++i) {
        for (int j=X.firstCol(); j<=X.lastCol(); ++j) {
            r += X(i,j)*X(i,j);
        }
    }
    return std::sqrt(h*r);
}

void
copy(const GridVector2D &x, GridVector2D &y)
{
    y.grid = x.grid;
}

void
scal(double alpha, GridVector2D &x)
{
    assert(0);
}

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
            out.operator<<(x);
            out << " ";
            out.operator<<(y);
            out << " ";
            out.operator<<(X(i,j));
            out << std::endl;
        }
        out << std::endl;
    }
    return out;
}

//-- Poisson2D -----------------------------------------------------------------

class Poisson2D;

template <>
struct TypeInfo<Poisson2D>
{
    typedef Poisson2D   Impl;
    typedef double      ElementType;
};

class Poisson2D
    : public SymmetricMatrix<Poisson2D>
{
    public:
        Poisson2D() {}

        Poisson2D(int _rh) : rh(_rh) {}

        int rh;
};

//-- Restriction ---------------------------------------------------------------

class Restriction;

template <>
struct TypeInfo<Restriction>
{
    typedef Restriction Impl;
    typedef double      ElementType;
};

class Restriction
    : public GeneralMatrix<Restriction>
{
};

template <typename ALPHA, typename VX, typename BETA, typename VY>
void
mv(Transpose trans, ALPHA alpha, const Restriction &A, const Vector<VX> &x,
   BETA beta, Vector<VY> &y)
{
    assert(trans==NoTrans);
    assert(alpha==ALPHA(1));
    assert(beta==BETA(0));

    mv(A, x.impl(), y.impl());
}

//-- Prolongation --------------------------------------------------------------

class Prolongation;

template <>
struct TypeInfo<Prolongation>
{
    typedef Prolongation Impl;
    typedef double       ElementType;
};

class Prolongation
    : public GeneralMatrix<Prolongation>
{
};

template <typename ALPHA, typename VX, typename BETA, typename VY>
void
mv(Transpose trans, ALPHA alpha, const Prolongation &A, const Vector<VX> &x,
   BETA beta, Vector<VY> &y)
{
    assert(trans==NoTrans);
    assert(alpha==ALPHA(1));
    assert(beta==BETA(1));

    mv(A, x.impl(), 1, y.impl());
}

//==============================================================================

/*
 *  compute:  y = alpha*A + beta*y
 *
 *  where   A    is the Poisson2D matrix
 *          x,y  are GridVector2D vectors
 */
template <typename ALPHA, typename BETA>
void
mv(ALPHA alpha, const Poisson2D &A, const GridVector2D &x,
   BETA beta, GridVector2D &y)
{
    assert(&x!=&y);

    const GridVector2D::Grid &X = x.grid;
    GridVector2D::Grid       &Y = y.grid;

    if ((Y.numRows()!=X.numRows()) || (Y.numCols()!=X.numCols())) {
        Y.resize(X.numRows(), X.numCols(),
                 X.firstRow(), X.firstCol());
    }

    int m = Y.lastRow()-1;
    int n = Y.lastCol()-1;

    alpha *= A.rh*A.rh;
    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            Y(i,j) = beta*Y(i,j)
                   + alpha*(4*X(i,j)-X(i-1,j)-X(i+1,j)-X(i,j-1)-X(i,j+1));
        }
    }
}

/*
 *  compute:  x = GS*x
 *
 *  where      x  is a GridVector2D vector
 *          GS*x  computes one Gauss-Seidel iteration for
 *                the system of linear equations  A*x = b
 */
void
mv(const GaussSeidel<Poisson2D, GridVector2D> &GS, GridVector2D &x)
{
    const GridVector2D::Grid &B = GS.b.grid;
    GridVector2D::Grid       &X = x.grid;

    int    m         = X.lastRow()-1;
    int    n         = X.lastCol()-1;
    double omega     = GS.omega;
    double c         = omega/4.;
    double hh        = 1./(GS.A.rh*GS.A.rh);

    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            X(i,j) = (1-omega)*X(i,j)
                   + c*(B(i,j)*hh + X(i-1,j) + X(i+1,j) + X(i,j-1) + X(i,j+1));
        }
    }
}

/*
 *  compute:  x = GS*x
 *
 *  where      x  is a GridVector2D vector
 *          GS*x  computes one Gauss-Seidel iteration for
 *                the system of linear equations  A*x = b
 */
void
mv(const GaussSeidel_RedBlack<Poisson2D, GridVector2D> &GS, GridVector2D &x)
{
    const GridVector2D::Grid &B = GS.b.grid;
    GridVector2D::Grid       &X = x.grid;

    int    m         = X.lastRow()-1;
    int    n         = X.lastCol()-1;
    double omega     = GS.omega;
    double c         = omega/4.;
    double hh        = 1./(GS.A.rh*GS.A.rh);


    // red nodes
    for (int i=1; i<=m; i+=2) {
        for (int j=1; j<=n; j+=2) {
            X(i,j) = (1-omega)*X(i,j)
                   + c*(B(i,j)*hh + X(i-1,j) + X(i+1,j) + X(i,j-1) + X(i,j+1));
        }
    }

    for (int i=2; i<=m; i+=2) {
        for (int j=2; j<=n; j+=2) {
            X(i,j) = (1-omega)*X(i,j)
                   + c*(B(i,j)*hh + X(i-1,j) + X(i+1,j) + X(i,j-1) + X(i,j+1));
        }
    }

    // black nodes
    for (int i=2; i<=m; i+=2) {
        for (int j=1; j<=n; j+=2) {
            X(i,j) = (1-omega)*X(i,j)
                   + c*(B(i,j)*hh + X(i-1,j) + X(i+1,j) + X(i,j-1) + X(i,j+1));
        }
    }

    for (int i=1; i<=m; i+=2) {
        for (int j=2; j<=n; j+=2) {
            X(i,j) = (1-omega)*X(i,j)
                   + c*(B(i,j)*hh + X(i-1,j) + X(i+1,j) + X(i,j-1) + X(i,j+1));
        }
    }
}

/*
 *  compute y = R*x
 *
 *  result:  y is restriction of x
 */
void
mv(const Restriction &R, const GridVector2D &x, GridVector2D &y)
{
    const GridVector2D::Grid &X = x.grid;
    GridVector2D::Grid &Y = y.grid;

    int    m         = Y.lastRow()-1;
    int    n         = Y.lastCol()-1;

    /*
    int I = 2;
    for (int i=1; i<=m; ++i, I+=2) {
        int J = 2;
        for (int j=1; j<=n; ++j, J+=2) {
            Y(i,j) = (   X(I-1,J-1) + 2*X(I-1,J) +   X(I-1,J+1)
                     + 2*X(I  ,J-1) + 4*X(I  ,J) + 2*X(I  ,J+1)
                     +   X(I+1,J-1) + 2*X(I+1,J) +   X(I+1,J+1))/16;
        }
    }
    */
    int I = 2;
    for (int i=1; i<=m; ++i, I+=2) {
        int J = 2;
        for (int j=1; j<=n; ++j, J+=2) {
            Y(i,j) = (                X(I-1,J)
                     + X(I  ,J-1) + 4*X(I  ,J) + X(I  ,J+1)
                                     +X(I+1,J))/8;
            //Y(i,j) = 0.5*X(I,J);
        }
    }
}

/*
 *  compute y += P*x
 *
 *  result:  y is updated by prolongation of x
 */
void
mv(const Prolongation &R, const GridVector2D &x, int beta, GridVector2D &y)
{
    assert(beta==1);

    const GridVector2D::Grid &X = x.grid;
    GridVector2D::Grid &Y = y.grid;

    int    m         = X.lastRow()-1;
    int    n         = X.lastCol()-1;

    int I = 2;
    for (int i=1; i<=m; ++i, I+=2) {
        int J = 2;
        for (int j=1; j<=n; ++j, J+=2) {
            Y(I-1,J-1)+=.25*X(i,j); Y(I-1,J)+=.5*X(i,j); Y(I-1,J+1)+=.25*X(i,j);
            Y(I  ,J-1)+=.50*X(i,j); Y(I  ,J)+=   X(i,j); Y(I  ,J+1)+=.50*X(i,j);
            Y(I+1,J-1)+=.25*X(i,j); Y(I+1,J)+=.5*X(i,j); Y(I+1,J+1)+=.25*X(i,j);
        }
    }

}

} // namespace flens












