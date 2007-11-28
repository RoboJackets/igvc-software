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

#ifndef FLENS_BLAS_H
#define FLENS_BLAS_H 1

#include <complex>

#include <flens/storage.h>

namespace flens {

//- Level 1 --------------------------------------------------------------------

void
copy(int N, const double *x, int incX, double *y, int incY);

template <typename T>
    void
    copy(int N, const T *x, int incX, T *y, int incY) __attribute__((deprecated));

void
scal(const int N, double alpha, double *x, int incX);

void
axpy(int N, double alpha, const double *x, int incX, double *y, int incY);

double
dot(int N, const double *x, int incX, const double *y, int incY);

double
nrm2(int N, const double *x, int incX);


int
amax(int N, const double *x, int incX);

int
amax(int N, const std::complex<double> *x, int incX);

// Level 1 Extentsions

int
amin(int N, const double *x, int incX);

int
amin(int N, const std::complex<double> *x, int incX);

int
asum(int N, const int *x, int incX);

// B = A
template <typename T>
void
copy(StorageOrder order, int M, int N, const T *A, int lda, T *B, int ldb);


//- Level 2 --------------------------------------------------------------------

void
gemv(StorageOrder order, Transpose trans,
     int m, int n, double alpha, const double *A, int lda,
     const double *x, int incX,
     double beta, double *y, int incY);

void
gbmv(StorageOrder order, Transpose trans,
     int m, int n, int kl, int ku, double alpha, const double *A, int lda,
     const double *x, int incX,
     double beta, double *y, int incY);

void
trmv(StorageOrder order, StorageUpLo upLo, Transpose trans, UnitDiag diag,
     int n, const double *A, int lda, double *x, int incX);

void
tbmv(StorageOrder order, StorageUpLo upLo, Transpose trans, UnitDiag diag,
     int n, int k, const double *A, const int lda, double *x, int incX);

void
tpmv(StorageOrder order, StorageUpLo upLo, Transpose trans, UnitDiag diag,
     int n, const double *A, double *x, int incX);

void
symv(StorageOrder order, StorageUpLo upLo,
     int n, double alpha, const double *A, int lda,
     const double *x, int incX,
     double beta, double *y, int incY);

void
sbmv(StorageOrder order, StorageUpLo upLo,
     int n, int k, double alpha, const double *A, int lda,
     const double *x, int incX,
     double beta, double *y, int incY);

void
spmv(StorageOrder order, StorageUpLo upLo,
     int n, double alpha, const double *A,
     const double *x, const int incX,
     double beta, double *y, const int incY);

//- Level 3 --------------------------------------------------------------------

void
gemm(StorageOrder order,
     Transpose transA, Transpose transB,
     int m, int n, int k, double alpha,
     const double *A, int lda,
     const double *B, int ldb,
     double beta, double *C, const int ldc);

enum BlasSide {
    Left,
    Right
};

void
symm(StorageOrder order, BlasSide side, StorageUpLo upLo,
     int m, int n, double alpha,
     const double *A, const int lda,
     const double *B, const int ldb,
     const double beta, double *C, const int ldc);

void
trmm(StorageOrder order, BlasSide side, StorageUpLo upLo,
     Transpose transA,
     UnitDiag diag, int m, int n,
     double alpha, const double *A, int lda,
     double *B, int ldb);

void
trsm(StorageOrder order, BlasSide side, StorageUpLo upLo,
     Transpose transA,
     UnitDiag diag, int m, int n,
     double alpha, const double *A, int lda,
     double *B, int ldb);

} // namespace flens

#include <flens/blas.tcc>

#endif // FLENS_BLAS_H
