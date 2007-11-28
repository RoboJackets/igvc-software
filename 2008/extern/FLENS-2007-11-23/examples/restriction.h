namespace flens {

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
mv(Transpose trans, ALPHA alpha, const Restriction &A, const DenseVector<VX> &x,
   BETA beta, DenseVector<VY> &y)
{
    assert(trans==NoTrans);
    assert(alpha==ALPHA(1));
    assert(beta==BETA(0));

    mv(A, x, y);
}

//------------------------------------------------------------------------------

template <typename VX, typename VY>
void
mv(const Restriction &R, const DenseVector<VX> &x, DenseVector<VY> &y)
{
    int n = y.lastIndex()-1;

    assert(x.firstIndex()<=1);
    assert(x.lastIndex()>=2*n+1);

    for (int i=1; i<=n; ++i) {
        y(i) = 0.25*(x(2*i-1) + 2*x(2*i) + x(2*i+1));
    }
}

} // namespace flens













