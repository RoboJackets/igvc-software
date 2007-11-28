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

#include <cmath>
#include <flens/blas.h>

#ifdef VECLIB
#    include <Accelerate/Accelerate.h>
#elif defined MKL
#    ifdef MAC
#        include <Intel_MKL/mkl_cblas.h>
#    else
#        include <mkl_cblas.h>
#    endif
#else
    extern "C" {
#        include <cblas.h>
    }
#endif

namespace flens {

// LEVEL 1
void
copy(int N, const double *x, int incX, double *y, int incY)
{
    cblas_dcopy(N, x, incX, y, incY);
}

void
scal(const int N, double alpha, double *X, int incX)
{
    cblas_dscal(N, alpha, X, incX);
}

void
axpy(int N, double alpha, const double *X, int incX, double *Y, int incY)
{
    cblas_daxpy(N, alpha, X, incX, Y, incY);
}

double
dot(int N, const double *X, int incX, const double *Y, int incY)
{
    return cblas_ddot(N, X, incX, Y, incY);
}

double
nrm2(int N, const double *X, int incX)
{
    return cblas_dnrm2(N, X, incX);
}

int
amax(int N, const double *X, int incX)
{
    return cblas_idamax(N, X, incX);
}

int
amax(int N, const std::complex<double> *X, int incX)
{
    return cblas_izamax(N, X, incX);
}

int
amin(int N, const double *X, int incX)
{
    int pos = 0;
    const double *max = X;
    for (int i=1; i<N; ++i) {
        if (std::abs(*(X+=incX))<std::abs(*max)) {
            max = X;
            pos = i;
        }
    }
    return pos;
}

int
amin(int N, const std::complex<double> *X, int incX)
{
    int pos = 0;
    const std::complex<double> *max = X;
    for (int i=1; i<N; ++i) {
        if (std::abs(*(X+=incX))<std::abs(*max)) {
            max = X;
            pos = i;
        }
    }
    return pos;
}

int
asum(int N, const int *x, int incX)
{
    int sum = 0;
    for (int i=0; i<N; i+=incX) {
        sum += x[i];
    }
    return sum;
}

// LEVEL 2
void
gemv(StorageOrder Order, Transpose Trans,
     int m, int n, double alpha, const double *A, int lda,
     const double *X, int incX,
     double beta, double *Y, int incY)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_TRANSPOSE trans = (Trans==NoTrans) ? CblasNoTrans : CblasTrans;

    cblas_dgemv(order, trans, m, n, alpha, A, lda, X, incX, beta, Y, incY);
}

void
gbmv(StorageOrder Order, Transpose Trans,
     int m, int n, int kl, int ku, double alpha, const double *A, int lda,
     const double *X, int incX,
     double beta, double *Y, int incY)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_TRANSPOSE trans = (Trans==NoTrans) ? CblasNoTrans : CblasTrans;

    cblas_dgbmv(order, trans, m, n, kl, ku,
                alpha, A, lda, X, incX, beta, Y, incY);
}

void
trmv(StorageOrder Order, StorageUpLo UpLo, Transpose Trans, UnitDiag Diag,
     int n, const double *A, int lda, double *X, int incX)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;
    CBLAS_TRANSPOSE trans = (Trans==NoTrans) ? CblasNoTrans : CblasTrans;
    CBLAS_DIAG diag = (Diag==NonUnit) ? CblasNonUnit : CblasUnit;

    cblas_dtrmv(order, upLo, trans, diag, n, A, lda, X, incX);
}

void
tbmv(StorageOrder Order, StorageUpLo UpLo, Transpose Trans, UnitDiag Diag,
     int n, int k, const double *A, const int lda, double *X, int incX)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;
    CBLAS_TRANSPOSE trans = (Trans==NoTrans) ? CblasNoTrans : CblasTrans;
    CBLAS_DIAG diag = (Diag==NonUnit) ? CblasNonUnit : CblasUnit;

    cblas_dtbmv(order, upLo, trans, diag, n, k, A, lda, X, incX);
}

void
tpmv(StorageOrder Order, StorageUpLo UpLo, Transpose Trans, UnitDiag Diag,
     int n, const double *A, double *x, int incX)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;
    CBLAS_TRANSPOSE trans = (Trans==NoTrans) ? CblasNoTrans : CblasTrans;
    CBLAS_DIAG diag = (Diag==NonUnit) ? CblasNonUnit : CblasUnit;

    cblas_dtpmv(order, upLo, trans, diag, n, A, x, incX);
}

void
symv(StorageOrder Order, StorageUpLo UpLo,
     int n, double alpha, const double *A, int lda,
     const double *X, int incX,
     double beta, double *Y, int incY)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;

    cblas_dsymv(order, upLo, n, alpha, A, lda, X, incX, beta, Y, incY);
}

void
sbmv(StorageOrder Order, StorageUpLo UpLo,
     int n, int k, double alpha, const double *A, int lda,
     const double *X, int incX,
     double beta, double *Y, int incY)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;

    cblas_dsbmv(order, upLo, n, k, alpha, A, lda, X, incX, beta, Y, incY);
}

void
spmv(StorageOrder Order, StorageUpLo UpLo,
     int n, double alpha, const double *A,
     const double *x, const int incX,
     double beta, double *y, const int incY)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;

    cblas_dspmv(order, upLo, n, alpha, A, x, incX, beta, y, incY);
}

// LEVEL 3
void
gemm(StorageOrder Order,
     Transpose TransA, Transpose TransB,
     int m, int n, int k, double alpha,
     const double *A, int lda,
     const double *B, int ldb,
     double beta, double *C, const int ldc)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_TRANSPOSE transA = (TransA==NoTrans) ? CblasNoTrans : CblasTrans;
    CBLAS_TRANSPOSE transB = (TransB==NoTrans) ? CblasNoTrans : CblasTrans;

    cblas_dgemm(order, transA, transB,
                m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
}

void
symm(StorageOrder Order, BlasSide Side, StorageUpLo UpLo,
     int m, int n, double alpha,
     const double *A, const int lda,
     const double *B, const int ldb,
     const double beta, double *C, const int ldc)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_SIDE side = (Side==Left) ? CblasLeft : CblasRight;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;

    cblas_dsymm(order, side, upLo, m, n, alpha, A, lda, B, ldb, beta, C, ldc);
}

void
trmm(StorageOrder Order, BlasSide Side, StorageUpLo UpLo,
     Transpose TransA,
     UnitDiag Diag, int m, int n,
     double alpha, const double *A, int lda,
     double *B, int ldb)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_SIDE side = (Side==Left) ? CblasLeft : CblasRight;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;
    CBLAS_TRANSPOSE transA = (TransA==NoTrans) ? CblasNoTrans : CblasTrans;
    CBLAS_DIAG diag = (Diag==NonUnit) ? CblasNonUnit : CblasUnit;

    cblas_dtrmm(order, side, upLo, transA, diag, m, n, alpha, A, lda, B, ldb);
}

void
trsm(StorageOrder Order, BlasSide Side, StorageUpLo UpLo,
     Transpose TransA,
     UnitDiag Diag, int m, int n,
     double alpha, const double *A, int lda,
     double *B, int ldb)
{
    CBLAS_ORDER order = (Order==RowMajor) ? CblasRowMajor : CblasColMajor;
    CBLAS_SIDE side = (Side==Left) ? CblasLeft : CblasRight;
    CBLAS_UPLO upLo = (UpLo==Upper) ? CblasUpper : CblasLower;
    CBLAS_TRANSPOSE transA = (TransA==NoTrans) ? CblasNoTrans : CblasTrans;
    CBLAS_DIAG diag = (Diag==NonUnit) ? CblasNonUnit : CblasUnit;

    cblas_dtrmm(order, side, upLo, transA, diag, m, n, alpha, A, lda, B, ldb);
}

} // namespace flens
