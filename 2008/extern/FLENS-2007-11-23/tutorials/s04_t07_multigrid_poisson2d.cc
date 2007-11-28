
#include <flens/flens.h>

//-- here our flens extensions ------------------
#include <tutorials/poisson2d.h>
#include <tutorials/gridvector2d.h>
#include <tutorials/gridvector2d_blas.h>
#include <tutorials/smoother_gs.h>
#include <tutorials/restriction.h>
#include <tutorials/prolongation.h>
#include <multigrid_vcycles.h>
//-----------------------------------------------

//-- problem set where exact solution is known --
#include <tutorials/problemset_poisson2d.h>
//-----------------------------------------------


using namespace flens;
using namespace std;

// number of grid levels
const long l = 6;

Poisson2D     A[l+1];
GridVector2D  f[l+1], r[l+1], u[l+1];

typedef MultiGrid<Poisson2D, GridVector2D, Restriction, Prolongation> MG;
typedef SmootherGS<Poisson2D, GridVector2D> Smoother;

int
main()
{
    SetupPoisson2DProblem  p(l, A, f, r, u);
    
    MG mg(A, f, r, u);
    
    p.displayStatistic(0);
    for (int k=2; k<=20; ++k) {
        mg.vCycle<Smoother>(l, 1, 1);
        p.displayStatistic(k);
    }
}
