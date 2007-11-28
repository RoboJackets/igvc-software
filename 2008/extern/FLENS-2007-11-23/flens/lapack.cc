/*
 *   Copyright (c) 2007, Michael Lehn
 *
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   1) Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2) Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3) Neither the name of the FLENS development group nor the names of
 *      its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cassert>
#include <complex>
#include <flens/lapack.h>

#ifdef VECLIB
#    include <Accelerate/Accelerate.h>
#elif defined MKL
#    ifdef MAC
#        include <Intel_MKL/mkl_lapack.h>
#    else
#        include <mkl_lapack.h>
#    endif
#else
#    include <cblas.h>
#endif

namespace flens {

extern "C" {

    void
    dgetrf_(int *m, int *n, double *a, int *lda, int *ipiv, int *info);

    void
    dgbtrf_(int *m, int *n, int *kl, int *ku,
            double *ab, int *ldab, int *ipiv, int *info);

    void
    dgetri_(int *n, double *a, int *lda, const int *ipiv, double *work,
            int *lwork, int *info);

    void
    dgetrs_(char *trans, int *n, int *nrhs, const double *a, int *lda,
            const int *ipiv, double *b, int *ldb, int *info);

    void
    dgbtrs_(char *trans, int *n, int *kl, int *ku, int *nrhs, const double *ab,
            int *ldab, const int *ipiv, double *b, int *ldb, int *info);

    void
    dgesv_(int *n, int *nrhs, double *a, int *lda,
           int *ipiv, double *b, int *ldb, int *info);

    void
    dgbsv_(int *n, int *kl, int *ku, int *nrhs, double *ab, int *ldab, int *ipiv,
           double *b, int *ldb, int *info);

    void
    dtrtrs_(char *uplo, char *trans, char *diag, int *n, int *nrhs,
            const double *a, int *lda, double *b, int *ldb, int *info);

    void
    dgeqrf_(int *m, int *n, double *a, int *lda, double *tau,
            double *work, int *lwork, int *info);

    void
    dorgqr_(int *m, int *n, int *k, double *a, int *lda, const double *tau,
            double *work, int *lwork, int *info);

    void
    dormqr_(char *side, char *trans, int *m, int *n, int *k,
            const double *a, int *lda, const double *tau, double *c, int *ldc,
            double *work, int *lwork, int *info);

    void
    dgels_(char *trans, int *m, int *n, int *nrhs, double *a, int *lda,
           double *b, int *ldb, double *work, int *lwork, int *info);

    void
    dgeev_(char *jobvl, char *jobvr, int *n, double *a, int *lda,
           double *wr, double *wi,
           double *vl, int *ldvl,
           double *vr, int *ldvr,
           double *work, int *lwork, int *info);

    void
    zgeev_(char *jobvl, char *jobvr, int *n, complex<double> *a, int *lda,
           complex<double> *w,
           complex<double> *vl, int *ldvl,
           complex<double> *vr, int *ldvr,
           complex<double> *work,int *lwork,double *rwork,int *info);

    void
    dgesvd_(char *jobu, char *jobvt, int *m, int *n, double *a, int *lda,
            double *s, double *u, int *ldu, double *vt, int *ldvt,
            double *work, int *lwork, int *info);
}

int
getrf(int m, int n, double *a, int lda, int *ipiv)
{
    int info;
    dgetrf_(&m, &n, a, &lda, ipiv, &info);
    return info;
}

int
gbtrf(int m, int n, int kl, int ku, double *ab, int ldab, int *ipiv)
{
    int info;
    dgbtrf_(&m, &n, &kl, &ku, ab, &ldab, ipiv, &info);
    return info;
}

int
getri(int n, double *a, int lda, const int *ipiv,
      double *work, int lwork)
{
    int info;
    dgetri_(&n, a, &lda, ipiv, work, &lwork, &info);
    return info;
}

int
getrs(Transpose trans, int n, int nrhs, const double *a, int lda,
      const int *ipiv, double *b, int ldb)
{
    int info;
    char _trans = (trans==NoTrans) ? 'N' : 'T';

    dgetrs_(&_trans, &n, &nrhs, a, &lda, ipiv, b, &ldb, &info);
    return info;
}

int
gbtrs(Transpose trans, int n, int kl, int ku, int nrhs,
      const double *ab, int ldab, const int *ipiv, double *b, int ldb)
{
    int info;
    char _trans = (trans==NoTrans) ? 'N' : 'T';
    dgbtrs_(&_trans, &n, &kl, &ku, &nrhs, ab, &ldab, ipiv, b, &ldb, &info);
    return info;
}

int
gesv(int n, int nrhs, double *a, int lda, int *ipiv, double *b, int ldb)
{
    int info;
    dgesv_(&n, &nrhs, a, &lda, ipiv, b, &ldb, &info);
    return info;
}

int
gbsv(int n, int kl, int ku, int nrhs, double *ab, int ldab,
     int *ipiv, double *b, int ldb)
{
    int info;
    dgbsv_(&n, &kl, &ku, &nrhs, ab, &ldab, ipiv, b, &ldb, &info);
    return info;
}

int
trtrs(StorageUpLo upLo, Transpose trans, UnitDiag diag, int n, int nrhs,
      const double *a, int lda, double *b, int ldb)
{
    int info;
    char _upLo = (upLo==Upper) ? 'U' : 'L';
    char _trans = (trans==NoTrans) ? 'N' : 'T';
    char _diag = (diag==Unit) ? 'U' : 'N';

    dtrtrs_(&_upLo, &_trans, &_diag, &n, &nrhs, a, &lda, b, &ldb, &info);
    return info;
}

int
geqrf(int m, int n, double *a, int lda, double *tau, double *work, int lwork)
{
    int info;
    dgeqrf_(&m, &n, a, &lda, tau, work, &lwork, &info);
    return info;
}

int
orgqr(int m, int n, int k, double *a, int lda, const double *tau,
      double *work, int lwork)
{
    int info;
    dorgqr_(&m, &n, &k, a, &lda, tau, work, &lwork, &info);
    return info;
}

int
ormqr(BlasSide side, Transpose trans, int m, int n, int k,
      const double *a, int lda, const double *tau, double *c, int ldc,
      double *work, int lwork)
{
    int info;
    char _side = (side==Left) ? 'L' : 'R';
    char _trans = (trans==NoTrans) ? 'N' : 'T';

    dormqr_(&_side, &_trans, &m, &n, &k, a, &lda, tau,
            c, &ldc, work, &lwork, &info);
    return info;
}

int
gels(Transpose trans, int m, int n, int nrhs, double *a, int lda,
     double *b, int ldb, double *work, int lwork)
{
    int info;
    char _trans = (trans==NoTrans) ? 'N' : 'T';

    dgels_(&_trans, &m, &n, &nrhs, a, &lda, b, &ldb, work, &lwork, &info);
    return info;
}

int
geev(bool jobvl, bool jobvr, int n, double *a, int lda,
     double *wr, double *wi,
     double *vl, int ldvl,
     double *vr, int ldvr,
     double *work, int lwork)
{
    int info;
    char _jobvl = (jobvl==true) ? 'V' : 'N';
    char _jobvr = (jobvr==true) ? 'V' : 'N';

    dgeev_(&_jobvl, &_jobvr, &n, a, &lda, wr, wi, vl, &ldvl, vr, &ldvr,
           work, &lwork, &info);
    return info;
}

int
geev(bool jobvl, bool jobvr, int n, complex<double> *a, int lda,
     complex<double> *w,
     complex<double> *vl, int ldvl,
     complex<double> *vr, int ldvr,
     complex<double> *work, int lwork, double *rwork)
{
    int info;
    char _jobvl = (jobvl==true) ? 'V' : 'N';
    char _jobvr = (jobvr==true) ? 'V' : 'N';

    zgeev_(&_jobvl, &_jobvr, &n, a, &lda, w, vl, &ldvl, vr, &ldvr,
           work, &lwork, rwork, &info);
    return info;
}

int
gesvd(char jobu, char jobvt,
      int m, int n, double *a, int lda,                // A
      double *s,                                      // singular values
      double *u, int ldu,                         // left singular vectors
      double *vt, int ldvt,                       // right singular vectors
      double *work, int lwork, int &info)
{
    dgesvd_(&jobu, &jobvt, &m, &n, a, &lda, s, u, &ldu, vt, &ldvt,
            work, &lwork, &info);
    return info;
}

} // namespace flens
