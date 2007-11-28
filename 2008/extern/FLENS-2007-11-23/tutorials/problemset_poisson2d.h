using namespace flens;
using namespace std;

struct SetupPoisson2DProblem
{
    int l, n;
    double h;

    Poisson2D *A;
    GridVector2D *f, *r, *u;
    GridVector2D solution, error;

    SetupPoisson2DProblem(int _l, Poisson2D *_A, GridVector2D *_f,
                          GridVector2D *_r, GridVector2D *_u)
        : l(_l), A(_A), f(_f), r(_r), u(_u)
    {
        // number of interior points: n = 2^l -1
        n = (1<<l) - 1;

        // mesh size
        h = 1./(n+1);

        // allocate all gridvectors
        solution = GridVector2D(n+1);
        error = GridVector2D(n+1);
        for (int L=l, N=n; L>=2; --L, N/=2) {
            u[L] = GridVector2D(N+1);
            f[L] = GridVector2D(N+1);
            r[L] = GridVector2D(N+1);
            A[L] = Poisson2D(N+1);
        }

        // set solution: sol(x,y)= (x^2-x^4)*(y^4-y^2)
        GridVector2D::Grid &X = solution.grid;
        for (int i=1; i<=n; ++i) {
            for (int j=1; j<=n; ++j) {
                double x = i*h;
                double y = j*h;

                X(i,j) = (x*x - x*x*x*x)*(y*y*y*y - y*y);
            }
        }

        // set right-hand side: f(x,y)= 2(1-6x^2)y^2(1-y^2) + (1-6y^2)x^2(1-x^2)
        GridVector2D::Grid &F = f[l].grid;
        for (int i=1; i<=n; ++i) {
            for (int j=1; j<=n; ++j) {
                double x = i*h;
                double y = j*h;

                F(i,j) = 2*((1-6*x*x)*y*y*(1-y*y) + (1-6*y*y)*x*x*(1-x*x));
            }
        }
    }

    void
    displayStatistic(int iteration)
    {
        error = solution - u[l];

        // compute discrete L_2 norms:
        r[l] = f[l] - A[l]*u[l];
        double residualNorm = h*sqrt(r[l]*r[l]);
        double errorNorm = h*sqrt(error*error);

        if (iteration==0) {
            cout << "#it  | ||r||_h  | ||e||_h  |" << endl;
            cout << "-----+----------+----------+" << endl;
        }

        cout.width(3);
        cout << iteration << ") | ";

        cout.precision(2);
        cout.setf(std::ios_base::scientific, ios_base::floatfield);
        cout.width(8);
        cout << residualNorm << " | ";

        cout.precision(2);
        cout.setf(std::ios_base::scientific, ios_base::floatfield);
        cout.width(8);
        cout << errorNorm << " | " << endl;
    }
};
