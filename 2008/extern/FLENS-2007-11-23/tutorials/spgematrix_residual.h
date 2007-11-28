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
    for (int i=1; i<=m; ++i) {
        rr = b[i];
        for (int k=ia[i]; k<ia[i+1]; ++k) {
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
