//#define DEBUG
//#define MATLAB_OUT
//#define BENCH

#include <tests/evaltest.h>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

namespace flens {

void
residual(int m, int n,
         const double *a, const int *ia, const int *ja,
         const double *x, const double *b, double *r)
{
    // shift to index base 1
    a = a-1;
    ia = ia-1;
    ja = ja-1;
    x = x-1;
    b = b-1;
    r = r-1;

    double rr;
    int begin, end;
    
    for (int i=1; i<=m; ++i) {
        rr = b[i];
        begin = ia[i];
        end = ia[i+1];
        for (int k=begin; k<end; ++k) {
            rr -= a[k]*x[ja[k]];
        }
        r[i] = rr;
    }
}

template <typename T, typename VX>
void
residual(const DenseVector<VX> &b,
         const SparseGeMatrix<CRS<T> > &A,
         const DenseVector<VX> &x,
         DenseVector<VX> &r)
{
    residual(A.numRows(), A.numCols(),
             A.engine().values.data(),
             A.engine().rows.data(),
             A.engine().columns.data(),
             x.data(),
             b.data(),
             r.data());
}

} // namespace flens

struct timer
{
    public:
        
        timer()
        {
            tic();
        }

        void
        tic()
        {
            _time = std::clock();
        }

        double
        toc() const
        {
            return  double(std::clock() - _time) / CLOCKS_PER_SEC;
        }

        std::clock_t _time;
};

void
test1()
{
    SparseGeMatrix<CRS<double> > A(10, 10);
    DenseVector<Array<double> >  x(10), y(10);

    int m = A.numRows();
    for (int i=1; i<=m; ++i) {
        if (i>1) {
            A(i,i-1) = 1;
        }
        A(i,i) =2;
        if (i<m) {
            A(i,i+1) = 3;
        }
        x(i) = 1;
    }
    A.finalize();

    DenseVector<Array<double> > sol(10);
    sol = 5, 6, 6, 6, 6, 6, 6, 6, 6, 3;

    mv(NoTrans, 1., A, x, 0., y);
    assertEqual(sol, y);

    y = A*x;
    assertEqual(sol, y);
}

void
test2()
{
    SparseGeMatrix<CRS<double> > A(10, 10);
    DenseVector<Array<double> >  x(10), y(10);

    int m = A.numRows();
    for (int i=1; i<=m; ++i) {
        if (i>1) {
            A(i,i-1) = 1;
        }
        A(i,i) =2;
        if (i<m) {
            A(i,i+1) = 3;
        }
        x(i) = i;
    }
    A.finalize();

    DenseVector<Array<double> > sol(10);
    sol = 8, 14, 20, 26, 32, 38, 44, 50, 56, 29;

    mv(NoTrans, 1., A, x, 0., y);
    assertEqual(sol, y);

    y = A*x;
    assertEqual(sol, y);
}

void
test3()
{
    SparseGeMatrix<CRS<double> > A(10, 10);
    DenseVector<Array<double> >  x(10), y(10);

    int m = A.numRows();
    for (int i=1; i<=m; ++i) {
        if (i>1) {
            A(i,i-1) = 1;
        }
        A(i,i) =2;
        if (i<m) {
            A(i,i+1) = 3;
        }
        x(i) = 1;
    }
    A.finalize();

    DenseVector<Array<double> > sol(10);
    sol = 3, 6, 6, 6, 6, 6, 6, 6, 6, 5;

    mv(Trans, 1., A, x, 0., y);
    assertEqual(sol, y);

    y = transpose(A)*x;
    assertEqual(sol, y);
}

void
test4()
{
    SparseGeMatrix<CRS<double> > A(10, 10);
    DenseVector<Array<double> >  x(10), y(10);

    int m = A.numRows();
    for (int i=1; i<=m; ++i) {
        if (i>1) {
            A(i,i-1) = 1;
        }
        A(i,i) =2;
        if (i<m) {
            A(i,i+1) = 3;
        }
        x(i) = i;
    }
    A.finalize();

    DenseVector<Array<double> > sol(10);
    sol = 4, 10, 16, 22, 28, 34, 40, 46, 52, 47;

    mv(Trans, 1., A, x, 0., y);
    assertEqual(sol, y);

    y = transpose(A)*x;
    assertEqual(sol, y);
}

void
test5()
{
    SparseGeMatrix<CRS<double> > A(20, 10);

    GeMatrix<FullStorage<int, RowMajor> > indices(20, 5);
    indices =  8,  2, 10,  7,  4,
               5,  6,  9,  1,  4,
               4,  1,  5,  7, 10,
               1,  9,  6,  3,  5,
               2, 10,  9,  4,  8,
               6,  3,  7,  5,  1,
               5,  1,  3,  7,  4,
              10,  2,  6,  9,  8,
               8,  9,  7,  1, 10,
               4,  6,  5,  3,  2,
               2,  5,  8,  7,  4,
              10,  9,  6,  1,  3,
               5,  8,  7,  3, 10,
               1,  9,  4,  2,  6,
               2,  9,  1,  4,  7,
               5,  8,  6,  3, 10,
               6,  4,  2,  5,  3,
               9,  7,  1, 10,  8,
              10,  8,  1,  2,  5,
               6,  9,  4,  3,  7;

    int m = A.numRows();
    int value = 3;
    for (int i=1; i<=m; ++i) {
        for (int k=1; k<=5; ++k) {
            A(i,indices(i,k)) = value;
            value += i*indices(i, 1+value%5)+i;
            value %= 53;
        }
        value %= 13;
    }
    A.finalize();

    /*
    typedef SparseGeMatrix<CRS<double> >::const_iterator It;
    for (It it=A.begin(); it!=A.end(); ++it) {
        cout << "A(" << it->first.first << ", " << it->first.second << ") = ";
        cout << it->second << ";" << std::endl;
    }
    */
    
    DenseVector<Array<double> >  x(10), y(20);
    for (int i=1; i<=10; ++i) {
        x(i) = value;
        value += i*indices(i, 1+value%5)+i;
        value %= 13;
    }

    DenseVector<Array<double> > sol(20);
    sol =  482,  768,  943,  801,  429, 1166,  976,  891,  442,  980,
           766,  767, 1558,  929,  845, 1315,  624, 1571, 1046,  708;

    mv(NoTrans, 1., A, x, 0., y);
    assertEqual(sol, y);

    y = A*x;
    assertEqual(sol, y);


    y = 482,  768,  943,  801,  429, 1166,  976,  891,  442,  980,
        766,  767, 1558,  929,  845, 1315,  624, 1571, 1046,  708;

    DenseVector<Array<double> > sol2(10);
    sol2 = 300117, 164777, 278723, 147800, 204893,
           117420, 313267, 276765, 173486, 165665;

    mv(Trans, 1., A, y, 0., x);
    assertEqual(sol2, x);

    x = transpose(A)*y;
    assertEqual(sol2, x);
}

void
test6()
{
    SparseSyMatrix<CRS<double, CRS_UpperTriangular> > A(10);
    DenseVector<Array<double> >  x(10), y(10);

    int m = A.dim();
    for (int i=1; i<=m; ++i) {
        A(i,i) =2;
        if (i<m) {
            A(i,i+1) = -1;
        }
        x(i) = 1;
    }
    A.finalize();

    DenseVector<Array<double> > sol(10);
    sol = 1, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    mv(1., A, x, 0., y);
    assertEqual(sol, y);

    y = A*x;
    assertEqual(sol, y);
}

void
test7()
{
    SparseSyMatrix<CRS<double, CRS_LowerTriangular> > A(10);
    DenseVector<Array<double> >  x(10), y(10);

    int m = A.dim();
    for (int i=1; i<=m; ++i) {
        A(i,i) =2;
        if (i<m) {
            A(i,i+1) = -1;
        }
        x(i) = 1;
    }
    A.finalize();

    DenseVector<Array<double> > sol(10);
    sol = 1, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    mv(1., A, x, 0., y);
    assertEqual(sol, y);

    y = A*x;
    assertEqual(sol, y);
}

void
test8()
{
    SparseSyMatrix<CRS<double, CRS_UpperTriangular> > A(10);
    DenseVector<Array<double> >  x(10), y(10);

    int m = A.dim();
    for (int i=1; i<=m; ++i) {
        A(i,i) =2;
        if (i<m) {
            A(i,i+1) = -1;
        }
        x(i) = i;
    }
    A.finalize();

    DenseVector<Array<double> > sol(10);
    sol = 0, 0, 0, 0, 0, 0, 0, 0, 0, 11;

    mv(1., A, x, 0., y);
    assertEqual(sol, y);

    y = A*x;
    assertEqual(sol, y);
}

void
test9()
{
    SparseSyMatrix<CRS<double, CRS_LowerTriangular> > A(10);
    DenseVector<Array<double> >  x(10), y(10);

    int m = A.dim();
    for (int i=1; i<=m; ++i) {
        A(i,i) =2;
        if (i<m) {
            A(i,i+1) = -1;
        }
        x(i) = i;
    }
    A.finalize();

    DenseVector<Array<double> > sol(10);
    sol = 0, 0, 0, 0, 0, 0, 0, 0, 0, 11;

    mv(1., A, x, 0., y);
    assertEqual(sol, y);

    y = A*x;
    assertEqual(sol, y);
}

void
test10()
{
#ifdef BENCH
    timer t;
    t.tic();
#endif // BENCH

    int n = 10;
    int k = 3;

#ifdef MATLAB_OUT
    int run = 1;
    std::ostringstream s;
    s << "test10_" << std::setw(3) << std::setfill('0') << run << ".m";
    std::ofstream out(s.str().c_str());
#endif // MATLAB_OUT

    SparseGeMatrix<CRS<double> > A(n, n);
    DenseVector<Array<double> >  x(n), y(n), sol(n), sol2(n);

#ifdef BENCH
    cerr << "allocated after " << t.toc() << endl;
    t.tic();
#endif // BENCH

#ifdef MATLAB_OUT
    out << "A1 = zeros(" << n << ", " << n << ");" << endl;
    out << "A = zeros(" << n << ", " << n << ");" << endl;
#endif // MATLAB_OUT
    for (int i=1; i<=n; ++i) {
        x(i) = 1;
        sol(i) = k;
        for (int j=1; j<=k; ++j) {
            int col = rand() % n +1;
            ++A(i,col);
#ifdef MATLAB_OUT
            out << "A1(" << i << ", " << col << ")  = "
                << "A1(" << i << ", " << col << ") + 1;"
                << endl;
#endif // MATLAB_OUT
            ++sol2(col);
        }
    }
#ifdef BENCH
    cerr << "initizalized after " << t.toc() << endl;
    t.tic();
#endif // BENCH

    A.finalize();

#ifdef BENCH
    cerr << "finalized after " << t.toc() << endl;
    t.tic();
#endif // BENCH

#ifdef MATLAB_OUT
    typedef SparseGeMatrix<CRS<double> >::const_iterator It;
    for (It it=A.begin(); it!=A.end(); ++it) {
        out << "A(" << it->first.first << ", " << it->first.second << ") = ";
        out << it->second << ";" << std::endl;
    }
    out << "max(max(A1-A))" << endl;
#endif // MATLAB_OUT

    for (int numIt=0; numIt<100; ++numIt) {
        mv(NoTrans, 1., A, x, 0., y);
        assertEqual(sol, y);

        y = A*x;
        assertEqual(sol, y);

        mv(Trans, 1., A, x, 0., y);
        assertEqual(sol2, y);

        y = transpose(A)*x;
        assertEqual(sol2, y);
    }

#ifdef BENCH
    cerr << "done after " << t.toc() << endl;
#endif // BENCH
}

void
test11()
{
#ifdef MATLAB_OUT
        std::ofstream out("test11.m");
#endif // MATLAB_OUT

#ifdef BENCH
    timer t;
    t.tic();
#endif // BENCH

    int n = 100;
    int k = 15;


    SparseSyMatrix<CRS<double, CRS_UpperTriangular> > A(n);
    DenseVector<Array<double> >  x(n), b(n), r(n), AUx(n), ALx(n), sol(n);


#ifdef MATLAB_OUT
        out << "A  = zeros(" << n << ", " << n << ");" << endl;
        out << "A1 = zeros(" << n << ", " << n << ");" << endl;
        out << "x = zeros(" << n << ", 1);" << endl;
        out << "r = zeros(" << n << ", 1);" << endl;
        out << "b = zeros(" << n << ", 1);" << endl;
        out << "sol = zeros(" << n << ", 1);" << endl;
#endif // MATLAB_OUT

    for (int i=1; i<=n; ++i) {
        int pos = rand() % n +1;
        int incr = rand() % n +1;
        x(pos) += incr;

        pos = rand() % n +1;
        incr = rand() % n +1;
        b(pos) += incr;
    }
    for (int i=1; i<=k*n; ++i) {
        int col = rand() % n +1;
        int row = rand() % n +1;
        int incr = rand() % n +1;

        A(row, col) += incr;
#ifdef MATLAB_OUT
            out << "A1(" << row << ", " << col << ")  = "
                << "A1(" << row << ", " << col << ") + "
                << incr << ";" << endl;
            if (row!=col) {
                out << "A1(" << col << ", " << row << ")  = "
                    << "A1(" << col << ", " << row << ") + "
                    << incr << ";" << endl;
            }
#endif // MATLAB_OUT

        AUx(row) += incr*x(col);
        if (row!=col) {
            ALx(col) += incr*x(row);
        }
    }
    sol = b -AUx -ALx;


#ifdef BENCH
    std::cerr << "initialization: " << t.toc() << "s" << std::endl;
    t.tic();
#endif // BENCH

    A.finalize();

#ifdef BENCH
    std::cerr << "finalization:   " << t.toc() << "s" << std::endl;
    t.tic();
#endif // BENCH

    for (int numIt=0; numIt<10; ++numIt) {
        r = b - A*x;
    }

#ifdef BENCH
    std::cerr << "computations:   " << t.toc() << "s" << std::endl;
#endif // BENCH

#ifdef MATLAB_OUT
    for (int i=1; i<=n; ++i) {
        out << "x(" << i << ") = " << x(i) << std::endl;
        out << "r(" << i << ") = " << r(i) << std::endl;
        out << "b(" << i << ") = " << b(i) << std::endl;
        out << "sol(" << i << ") = " << sol(i) << std::endl;
    }
    typedef SparseSyMatrix<CRS<double, CRS_LowerTriangular> >::const_iterator It;
    for (It it=A.begin(); it!=A.end(); ++it) {
        out << "A(" << it->first.first << ", " << it->first.second << ") = ";
        out << it->second << ";" << std::endl;
        out << "A(" << it->first.second << ", " << it->first.first << ") = ";
        out << it->second << ";" << std::endl;
    }
    out << "max(max(A1-A))" << endl;
#endif // MATLAB_OUT
    assertEqual(sol,r);
}

void
test12()
{
#ifdef MATLAB_OUT
        std::ofstream out("test12.m");
#endif // MATLAB_OUT

#ifdef BENCH
    timer t;
    t.tic();
#endif // BENCH

    int n = 100;
    int k = 15;


    SparseSyMatrix<CRS<double, CRS_LowerTriangular> > A(n);
    DenseVector<Array<double> >  x(n), b(n), r(n), AUx(n), ALx(n), sol(n);


#ifdef MATLAB_OUT
        out << "A  = zeros(" << n << ", " << n << ");" << endl;
        out << "A1 = zeros(" << n << ", " << n << ");" << endl;
        out << "x = zeros(" << n << ", 1);" << endl;
        out << "r = zeros(" << n << ", 1);" << endl;
        out << "b = zeros(" << n << ", 1);" << endl;
        out << "sol = zeros(" << n << ", 1);" << endl;
#endif // MATLAB_OUT

    for (int i=1; i<=n; ++i) {
        int pos = rand() % n +1;
        int incr = rand() % n +1;
        x(pos) += incr;

        pos = rand() % n +1;
        incr = rand() % n +1;
        b(pos) += incr;
    }
    for (int i=1; i<=k*n; ++i) {
        int col = rand() % n +1;
        int row = rand() % n +1;
        int incr = rand() % n +1;

        A(row, col) += incr;
#ifdef MATLAB_OUT
            out << "A1(" << row << ", " << col << ")  = "
                << "A1(" << row << ", " << col << ") + "
                << incr << ";" << endl;
            if (row!=col) {
                out << "A1(" << col << ", " << row << ")  = "
                    << "A1(" << col << ", " << row << ") + "
                    << incr << ";" << endl;
            }
#endif // MATLAB_OUT

        AUx(row) += incr*x(col);
        if (row!=col) {
            ALx(col) += incr*x(row);
        }
    }
    sol = b -AUx -ALx;

#ifdef BENCH
    std::cerr << "initialization: " << t.toc() << "s" << std::endl;
    t.tic();
#endif // BENCH

    A.finalize();

#ifdef BENCH
    std::cerr << "finalization:   " << t.toc() << "s" << std::endl;
    t.tic();
#endif // BENCH

    for (int numIt=0; numIt<10; ++numIt) {
        r = b - A*x;
    }
#ifdef BENCH
    std::cerr << "computations:   " << t.toc() << "s" << std::endl;
#endif // BENCH

#ifdef MATLAB_OUT
    for (int i=1; i<=n; ++i) {
        out << "x(" << i << ") = " << x(i) << std::endl;
        out << "r(" << i << ") = " << r(i) << std::endl;
        out << "b(" << i << ") = " << b(i) << std::endl;
        out << "sol(" << i << ") = " << sol(i) << std::endl;
    }
    typedef SparseSyMatrix<CRS<double, CRS_LowerTriangular> >::const_iterator It;
    for (It it=A.begin(); it!=A.end(); ++it) {
        out << "A(" << it->first.first << ", " << it->first.second << ") = ";
        out << it->second << ";" << std::endl;
        out << "A(" << it->first.second << ", " << it->first.first << ") = ";
        out << it->second << ";" << std::endl;
    }
    out << "max(max(A1-A))" << endl;
#endif // MATLAB_OUT
    assertEqual(sol,r);
}


void
test13()
{
#ifdef BENCH
    timer t;
    t.tic();
#endif // BENCH

#ifdef MATLAB_OUT
    std::ofstream cmd("test11_run.m");
#endif // MATLAB_OUT

    int n = 1000000;
    int k = 5;
    for (int run = 1; run<=10; ++run, n/=2, k+=2) {
#ifdef MATLAB_OUT
        std::ostringstream s;
        s << "test13_" << std::setw(3) << std::setfill('0') << run << ".m";
        std::ofstream out(s.str().c_str());

        cmd << "test13_" << std::setw(3) << std::setfill('0') << run << endl;
#endif // MATLAB_OUT

        SparseGeMatrix<CRS<double> > A(n, n, k);
        DenseVector<Array<double> >  x(n), y(n), sol(n), sol2(n);

#ifdef BENCH
        cerr << "allocated after " << t.toc() << endl;
        t.tic();
#endif // BENCH

#ifdef MATLAB_OUT
        out << "A1 = zeros(" << n << ", " << n << ");" << endl;
        out << "A = zeros(" << n << ", " << n << ");" << endl;
#endif // MATLAB_OUT
        for (int i=1; i<=n; ++i) {
            x(i) = 1;
        }
        for (int i=1; i<=k*n; ++i) {
            int col = rand() % n +1;
            int row = rand() % n +1;
            int incr = rand() % n +1;

            A(row, col) += incr;
#ifdef MATLAB_OUT
            out << "A1(" << row << ", " << col << ")  = "
                << "A1(" << row << ", " << col << ") + "
                << incr << ";" << endl;
#endif // MATLAB_OUT
            sol(row) += incr;
            sol2(col) += incr;
        }
#ifdef BENCH
        cerr << "initizalized after " << t.toc() << endl;
        t.tic();
#endif // BENCH

        A.finalize();

#ifdef BENCH
        cerr << "finalized after " << t.toc() << endl;
        t.tic();
#endif // BENCH

#ifdef MATLAB_OUT
        typedef SparseGeMatrix<CRS<double> >::const_iterator It;
#ifdef DEBUG
        for (It it=A.begin(); it!=A.end(); ++it) {
            cerr << "A(" << it->first.first << ", " << it->first.second << ") = ";
            cerr << it->second << ";" << std::endl;
        }
#endif // DEBUG
        for (It it=A.begin(); it!=A.end(); ++it) {
            out << "A(" << it->first.first << ", " << it->first.second << ") = ";
            out << it->second << ";" << std::endl;
        }
        out << "max(max(A1-A))" << endl;
#endif // MATLAB_OUT

#ifdef BENCH
        for (int numIt=0; numIt<10; ++numIt) {
#endif // BENCH
            mv(NoTrans, 1., A, x, 0., y);
            assertEqual(sol, y);

            y = A*x;
            assertEqual(sol, y);

            mv(Trans, 1., A, x, 0., y);
            assertEqual(sol2, y);

            y = transpose(A)*x;
            assertEqual(sol2, y);
#ifdef BENCH
        }
#endif // BENCH

#ifdef BENCH
        cerr << "done after " << t.toc() << endl;
#endif // BENCH
    }
}

void
test14()
{
#ifdef BENCH
    timer t;
    t.tic();
#endif // BENCH

#ifdef MATLAB_OUT
    std::ofstream cmd("test11_run.m");
#endif // MATLAB_OUT

    int n = 100;
    int k = 5;
    for (int run = 1; run<=10; ++run, k+=15) {
#ifdef MATLAB_OUT
        std::ostringstream s;
        s << "test12_" << std::setw(3) << std::setfill('0') << run << ".m";
        std::ofstream out(s.str().c_str());

        cmd << "test12_" << std::setw(3) << std::setfill('0') << run << endl;
#endif // MATLAB_OUT

        SparseGeMatrix<CRS<double> > A(n, n, k);
        DenseVector<Array<double> >  x(n), y(n), sol(n), sol2(n);

#ifdef BENCH
        cerr << "allocated after " << t.toc() << endl;
        t.tic();
#endif // BENCH

#ifdef MATLAB_OUT
        out << "A1 = zeros(" << n << ", " << n << ");" << endl;
        out << "A = zeros(" << n << ", " << n << ");" << endl;
#endif // MATLAB_OUT
        for (int i=1; i<=n; ++i) {
            x(i) = 1;
        }
        for (int i=1; i<=k*n; ++i) {
            int col = rand() % n +1;
            int row = rand() % n +1;
            int incr = rand() % n +1;

            A(row, col) += incr;
#ifdef MATLAB_OUT
            out << "A1(" << row << ", " << col << ")  = "
                << "A1(" << row << ", " << col << ") + "
                << incr << ";" << endl;
#endif // MATLAB_OUT
            sol(row) += incr;
            sol2(col) += incr;
        }
#ifdef BENCH
        cerr << "initizalized after " << t.toc() << endl;
        t.tic();
#endif // BENCH

        A.finalize();

#ifdef BENCH
        cerr << "finalized after " << t.toc() << endl;
        t.tic();
#endif // BENCH

#ifdef MATLAB_OUT
        typedef SparseGeMatrix<CRS<double> >::const_iterator It;
#ifdef DEBUG
        for (It it=A.begin(); it!=A.end(); ++it) {
            cerr << "A(" << it->first.first << ", " << it->first.second << ") = ";
            cerr << it->second << ";" << std::endl;
        }
#endif // DEBUG
        for (It it=A.begin(); it!=A.end(); ++it) {
            out << "A(" << it->first.first << ", " << it->first.second << ") = ";
            out << it->second << ";" << std::endl;
        }
        out << "max(max(A1-A))" << endl;
#endif // MATLAB_OUT

#ifdef BENCH
        for (int numIt=0; numIt<10; ++numIt) {
#endif // BENCH
            mv(NoTrans, 1., A, x, 0., y);
            assertEqual(sol, y);

            y = A*x;
            assertEqual(sol, y);

            mv(Trans, 1., A, x, 0., y);
            assertEqual(sol2, y);

            y = transpose(A)*x;
            assertEqual(sol2, y);
#ifdef BENCH
        }
#endif // BENCH

#ifdef BENCH
        cerr << "done after " << t.toc() << endl;
#endif // BENCH
    }
}

void
test15()
{
#ifdef BENCH
    timer t;
    t.tic();
#endif // BENCH

    int n = 1000000;

    SparseGeMatrix<CRS<double> > A(n, n, 3);
    DenseVector<Array<double> >  x(n), b(n), r(n), Ax(n), sol(n);

#ifdef BENCH
    cerr << "allocated after " << t.toc() << endl;
    t.tic();
#endif // BENCH

    for (int i=1; i<=n; ++i) {
        x(i) = rand() % n +1;
        b(i) = rand() % n +1;
    }
    for (int i=1; i<=n; ++i) {
        double a = -1,
               b =  2,
               c = -1;
        
        if (i>1) {
            A(i,i-1) = a;
            Ax(i) += a*x(i-1);
        }
        A(i,i) = b;
        Ax(i) += b*x(i);
        if (i<n) {
            A(i,i+1) = -1;
            Ax(i) += c*x(i+1);
        }
    }
#ifdef BENCH
    cerr << "initizalized after " << t.toc() << endl;
#endif // BENCH

#ifdef BENCH
    t.tic();
#endif // BENCH

    A.finalize();

#ifdef BENCH
    cerr << "finalized after " << t.toc() << endl;
    t.tic();
#endif // BENCH

    for (int k=1; k<=20; ++k) {
        r = A*x - b;
    }

#ifdef BENCH
    cerr << "computed: r = A*x - b after " << t.toc() << endl;
#endif // BENCH

    sol = Ax - b;
    assertEqual(sol, r);

#ifdef BENCH
    t.tic();
#endif // BENCH
    for (int k=1; k<=20; ++k) {
        r = b - A*x;
    }
#ifdef BENCH
    cerr << "computed: r = b - A*x after " << t.toc() << endl;
#endif // BENCH
    sol = b - Ax;
    assertEqual(sol, r);

#ifdef BENCH
    t.tic();
#endif // BENCH
    for (int k=1; k<=20; ++k) {
        residual(b, A, x, r);
    }
#ifdef BENCH
    cerr << "computed: residual(A, x, b, r) after " << t.toc() << endl;
#endif // BENCH
    sol = b - Ax;
    assertEqual(sol, r);
}

void
test()
{
    int n = 3;
    cerr << "n = " << n << endl;

    SparseGeMatrix<CRS<double> > A(n, n);

    A(2,1) += 2;
    A(2,2) += 2;
    A(2,3) += 2;
    A(1,1) += 2;
    A(1,2) += 2;
    A(1,2) += 2;
    A(2,1) += 2;
    A(2,1) += 2;
    A(2,1) += 2;
    A(3,2) += 2;
    A(3,3) += 2;
    A(3,3) += 2;
    A.finalize();

    cout << "A = " << A << endl;
}

int
main()
{
    //test();
    run("test1", test1, true);
    run("test2", test2, true);
    run("test3", test3, true);
    run("test4", test4, true);
    run("test5", test5, true);
    run("test6", test6, true);
    run("test7", test7, true);
    run("test8", test8, true);
    run("test9", test9, true);
    run("test10", test10, true);
    run("test11", test11, true);
    run("test12", test12, true);
    run("test13", test13, true);
    run("test14", test14, true);
    run("test15", test15, true);
}
