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

#include <flens/lapack.h>

namespace flens {

//-- getrf ---------------------------------------------------------------------

template <typename FS>
int
trf(GeMatrix<FS> &A, DenseVector<Array<int> > &P)
{
    return getrf(A.numRows(), A.numCols(),
                 A.data(), A.leadingDimension(),
                 P.data());
}

//-- gbtrf ---------------------------------------------------------------------

template <typename BS>
int
trf(GbMatrix<BS> &A, DenseVector<Array<int> > &P)
{
    return gbtrf(A.numRows(), A.numCols(),
                 A.numSubDiags(), A.numSuperDiags()-A.numSubDiags(),
                 A.data(), A.leadingDimension(),
                 P.data());
}

//-- getri ---------------------------------------------------------------------

template <typename MA>
int
tri(GeMatrix<MA> &A, DenseVector<Array<int> > &P)
{
    assert(A.numRows()==A.numCols());
    assert(P.length()==A.numCols());

    typedef typename MA::ElementType T;
    T lwork;
    int info;

    // query optimal work space size
    info = getri(A.numCols(), A.data(), A.leadingDimension(), P.data(),
                 &lwork, -1);
    assert(info==0);

    // allocate work space
    DenseVector<Array<T> > work((int) lwork);
    return getri(A.numCols(), A.data(), A.leadingDimension(), P.data(),
                 work.data(), work.length());
}

//-- getrs ---------------------------------------------------------------------

template <typename MA, typename MB>
int
trs(Transpose trans, const GeMatrix<MA> &LU,
    const DenseVector<Array<int> > &P, GeMatrix<MB> &B)
{
    assert(LU.numRows()==LU.numCols());
    assert(P.length()==LU.numRows());

    return getrs(trans, LU.numCols(), B.numCols(),
                 LU.data(), LU.leadingDimension(),
                 P.data(), B.data(), B.leadingDimension());
}

template <typename MA, typename VB>
int
trs(Transpose trans, const GeMatrix<MA> &LU,
    const DenseVector<Array<int> > &P,
    DenseVector<VB> &B)
{
    assert(LU.numRows()==LU.numCols());
    assert(P.length()==LU.numRows());
    assert(B.stride()==1);

    return getrs(trans, LU.numCols(), 1,
                 LU.data(), LU.leadingDimension(),
                 P.data(), B.data(), B.length());
}

//-- gbtrs ---------------------------------------------------------------------

template <typename MA, typename MB>
int
trs(Transpose trans, GbMatrix<MA> &LU,
    const DenseVector<Array<int> > &P, GeMatrix<MB> &B)
{
    assert(LU.numRows()==LU.numCols());
    assert(P.length()==LU.numRows());

    return gbtrs(trans, LU.numRows(),
                 LU.numSubDiags(), LU.numSuperDiags()-LU.numSubDiags(),
                 B.numCols(), LU.data(), LU.leadingDimension(),
                 P.data(), B.data(), B.leadingDimension());
}

template <typename MA, typename VB>
int
trs(Transpose trans, const GbMatrix<MA> &LU,
    const DenseVector<Array<int> > &P, DenseVector<VB> &B)
{
    assert(LU.numRows()==LU.numCols());
    assert(P.length()==LU.numRows());
    assert(B.stride()==1);

    return gbtrs(trans, LU.numRows(),
                 LU.numSubDiags(), LU.numSuperDiags()-LU.numSubDiags(),
                 1, LU.data(), LU.leadingDimension(),
                 P.data(), B.data(), B.length());
}

//-- gesv ----------------------------------------------------------------------

template <typename MA, typename MB>
int
sv(GeMatrix<MA> &A, DenseVector<Array<int> > &P, GeMatrix<MB> &B)
{
    assert(A.numRows()==A.numCols());
    assert(B.numRows()==A.numRows());

    return gesv(A.numRows(), B.numCols(), A.data(), A.leadingDimension(),
                P.data(), B.data(), B.leadingDimension());
}

template <typename MA, typename VB>
int
sv(GeMatrix<MA> &A, DenseVector<Array<int> > &P, DenseVector<VB> &B)
{
    assert(A.numRows()==A.numCols());
    assert(B.length()==A.numRows());
    assert(B.stride()==1);

    return gesv(A.numRows(), 1, A.data(), A.leadingDimension(),
                P.data(), B.data(), B.length());
}

//-- gbsv ----------------------------------------------------------------------

template <typename MA, typename MB>
int
sv(GbMatrix<MA> &A, DenseVector<Array<int> > &P, GeMatrix<MB> &B)
{
    assert(A.numRows()==A.numCols());

    return gbsv(A.numRows(),
                A.numSubDiags(), A.numSuperDiags()-A.numSubDiags(),
                B.numCols(),
                A.data(), A.leadingDimension(),
                P.data(), B.data(), B.leadingDimension());
}

template <typename MA, typename VB>
int
sv(GbMatrix<MA> &A, DenseVector<Array<int> > &P, DenseVector<VB> &B)
{
    assert(A.numRows()==A.numCols());

    return gbsv(A.numRows(),
                A.numSubDiags(), A.numSuperDiags()-A.numSubDiags(),
                1,
                A.data(), A.leadingDimension(),
                P.data(), B.data(), B.length());
}

//-- trtrs ---------------------------------------------------------------------

template <typename MA, typename MB>
int
trs(Transpose trans, const TrMatrix<MA> &A, GeMatrix<MB> &B)
{
    return trtrs(A.upLo(), trans, A.unitDiag(), A.dim(), B.numCols(),
                 A.data(), A.leadingDimension(),
                 B.data(), B.leadingDimension());
}

template <typename MA, typename VB>
int
trs(Transpose trans, const TrMatrix<MA> &A, DenseVector<VB> &B)
{
    assert(B.stride()==1);

    return trtrs(A.upLo(), trans, A.unitDiag(), A.dim(), 1,
                 A.data(), A.leadingDimension(), B.data(), B.length());
}

//-- geqrf ---------------------------------------------------------------------

template <typename MA, typename VT>
int
qrf(GeMatrix<MA> &A, DenseVector<VT> &tau)
{
    typedef typename MA::ElementType T;
    T lwork;
    int info;
    tau.resize(std::min(A.numRows(),A.numCols()));

    // query optimal work space size
    info = geqrf(A.numRows(), A.numCols(),
                 A.data(), A.leadingDimension(), tau.data(),
                 &lwork, -1);
    assert(info==0);

    // allocate work space
    DenseVector<Array<T> > work((int) lwork);
    return geqrf(A.numRows(), A.numCols(), A.data(), A.leadingDimension(),
                 tau.data(), work.data(), work.length());
}

//-- orgqr ---------------------------------------------------------------------

template <typename MA, typename VT>
int
orgqr(GeMatrix<MA> &A, const DenseVector<VT> &tau)
{
    typedef typename MA::ElementType T;
    T lwork;
    int info;

    // query optimal work space size
    info = orgqr(A.numRows(), A.numCols(), tau.length(),
                 A.data(), A.leadingDimension(), tau.data(),
                 &lwork, -1);
    assert(info==0);

    // allocate work space
    DenseVector<Array<T> > work((int) lwork);
    return orgqr(A.numRows(), A.numCols(), tau.length(),
                 A.data(), A.leadingDimension(), tau.data(),
                 work.data(), work.length());
}

//-- ormqr ---------------------------------------------------------------------

template <typename MA, typename VT, typename MC>
int
ormqr(BlasSide side, Transpose trans,
      const GeMatrix<MA> &A, const DenseVector<VT> &tau,
      GeMatrix<MC> &C)
{
    typedef typename MA::ElementType T;
    T lwork;
    int info;

    // query optimal work space size
    info = ormqr(side, trans, C.numRows(), C.numCols(), tau.length(),
                 A.data(), A.leadingDimension(), tau.data(),
                 C.data(), C.leadingDimension(),
                 &lwork, -1);
    assert(info==0);

    // allocate work space
    DenseVector<Array<T> > work((int) lwork);
    return ormqr(side, trans, C.numRows(), C.numCols(), tau.length(),
                 A.data(), A.leadingDimension(), tau.data(),
                 C.data(), C.leadingDimension(),
                 work.data(), work.length());
}

//-- gels ----------------------------------------------------------------------

template <typename MA, typename MB>
int
ls(Transpose trans, GeMatrix<MA> &A, GeMatrix<MB> &B)
{
    typedef typename MA::ElementType T;
    T lwork;
    int info;

    // query work space size
    info = gels(trans, A.numRows(), A.numCols(), B.numCols(),
                A.data(), A.leadingDimension(),
                B.data(), B.leadingDimension(),
                &lwork, -1);
    assert(info==0);

    // allocate work space
    DenseVector<Array<T> > work((int) lwork);
    return gels(trans, A.numRows(), A.numCols(), B.numCols(),
                A.data(), A.leadingDimension(),
                B.data(), B.leadingDimension(),
                work.data(), work.length());
}

//-- geev ----------------------------------------------------------------------

template <typename MA, typename WR, typename WI, typename VL, typename VR>
int
ev(bool leftEV, bool rightEV,
   GeMatrix<MA> &A, DenseVector<WR> &wr, DenseVector<WI> &wi,
   GeMatrix<VL> &vl, GeMatrix<VR> &vr)
{
    assert(A.numRows()==A.numCols());
    assert(wr.length()==A.numRows());
    assert(wi.length()==A.numRows());
    assert(vl.numRows()==vl.numCols());
    assert(vr.numRows()==vr.numCols());
    assert(!leftEV || (vl.numRows()==A.numRows()));
    assert(!rightEV || (vr.numRows()==A.numRows()));

    int ldvl = leftEV  ? vl.engine().leadingDimension() : 1;
    int ldvr = rightEV ? vr.engine().leadingDimension() : 1;
    double *vldata = leftEV  ? vl.data() : 0;
    double *vrdata = rightEV ? vr.data() : 0;
    typedef typename MA::ElementType T;
    T lwork;
    int info;

    // query optimal work space size
    info = geev(leftEV, rightEV, A.numRows(), A.data(), A.leadingDimension(),
                wr.data(), wi.data(),
                vldata, ldvl,
                vrdata, ldvr,
                &lwork, -1);
    assert(info==0);

    // allocate work space
    DenseVector<Array<T> > work((int) lwork);
    return geev(leftEV, rightEV, A.numRows(), A.data(), A.leadingDimension(),
                wr.data(), wi.data(),
                vldata, ldvl,
                vrdata, ldvr,
                work.data(), work.length());
}

template <typename MA, typename W, typename VL, typename VR>
int
ev(bool leftEV, bool rightEV,
   GeMatrix<MA> &A, DenseVector<W> &w, GeMatrix<VL> &vl, GeMatrix<VR> &vr)
{
    assert(A.numRows()==A.numCols());
    assert(w.length()==A.numRows());
    assert(vl.numRows()==vl.numCols());
    assert(vr.numRows()==vr.numCols());
    assert(!leftEV || (vl.numRows()==A.numRows()));
    assert(!rightEV || (vr.numRows()==A.numRows()));

    int ldvl = leftEV  ? vl.engine().leadingDimension() : 1;
    int ldvr = rightEV ? vr.engine().leadingDimension() : 1;
    typename VL::ElementType *vldata = leftEV  ? vl.data() : 0;
    typename VR::ElementType *vrdata = rightEV ? vr.data() : 0;
    typedef typename MA::ElementType T;
    T lwork;
    DenseVector<Array<typename T::value_type> > rwork(2*A.numRows());
    int info;

    // query optimal work space size
    info = geev(leftEV, rightEV, A.numRows(), A.data(), A.leadingDimension(),
                w.data(),
                vldata, ldvl,
                vrdata, ldvr,
                &lwork, -1, rwork.data());
    assert(info==0);

    // allocate work space
    DenseVector<Array<T> > work((int) lwork.real());
    return geev(leftEV, rightEV, A.numRows(), A.data(), A.leadingDimension(),
                w.data(),
                vldata, ldvl,
                vrdata, ldvr,
                work.data(), work.length(), rwork.data());
}

} // namespace flens
