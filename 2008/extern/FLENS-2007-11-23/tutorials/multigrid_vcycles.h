#ifndef TUTORIALS_MULTIGRID_VCYCLES_H
#define TUTORIALS_MULTIGRID_VCYCLES_H 1

namespace flens {

template <typename SystemMatrix, typename VectorType,
          typename Restrict, typename Prolongate>
struct MultiGrid
{

    const SystemMatrix  *A;
    VectorType          *f, *r, *u;

    Restriction     R;
    Prolongation    P;

    MultiGrid(const SystemMatrix *_A,
              VectorType *_f, VectorType *_r, VectorType *_u)
        : A(_A), f(_f), r(_r), u(_u)
    {
    }

    template <typename S>
    void
    vCycle(int l, int v1, int v2)
    {
        if (l==2) {
            for (int k=1; k<=20; ++k) {
                u[l] = S(A[l], f[l])*u[l];
            }
        } else {
            for (int v=1; v<=v1; ++v) {
                u[l] = S(A[l], f[l])*u[l];
            }
            r[l] = f[l] - A[l]*u[l];
            f[l-1] = R*r[l];

            u[l-1] = 0;
            vCycle<S>(l-1,v1,v2);

            u[l] += P*u[l-1];
            for (int v=1; v<=v2; ++v) {
                u[l] = S(A[l], f[l])*u[l];
            }
        }
    }
};

} // namespace flens

#endif // TUTORIALS_MULTIGRID_VCYCLES_H
