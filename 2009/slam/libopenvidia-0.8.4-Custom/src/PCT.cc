/**
 * This file is part of the OpenVIDIA project at http://openvidia.sf.net
 * Copyright (C) 2004, James Fung
 *
 * OpenVIDIA is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * OpenVIDIA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OpenVIDIA; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 **/


//#ifdef __cplusplus
//extern "C" {
//#endif



#include "mat_util.h"
#include "pinverse.h"
#include "corners2r.h"
#include "corners2d.h"
#include "motion.h" //for pcompose
//#ifdef __cplusplus
//}
//#endif

#include <stdlib.h>
#include <iostream>
using namespace std;

#include <math.h>
#include <openvidia/tnt/tnt.h>
#include <openvidia/tnt/jama_svd.h>
#include <openvidia/openvidia32.h>

using namespace TNT;
using namespace JAMA;


double dist( Match &m) {
    double x1 = m.first->x();
    double y1 = m.first->y();

    double x2 = m.second->x();
    double y2 = m.second->y();

    return ((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

PCT::PCT()
{
    for ( int i = 0; i<8 ; i++ ) {
        push_back(0.0);
    }
    (*this)[0] = 1.0;
    (*this)[4] = 1.0;
}

void PCT::set( double a11, double a12, double b1,
               double a21, double a22, double b2,
               double c1,  double c2 )
{
    clear();
    push_back(a11);
    push_back(a12);
    push_back(b1);
    push_back(a21);
    push_back(a22);
    push_back(b2);
    push_back(c1);
    push_back(c2);
}

PCT::PCT( float a11in, float a12in, float b1in, float a21in,
          float a22in, float b2in,  float c1in, float c2in )
{
    clear();
    push_back(a11in);
    push_back(a12in);
    push_back(b1in);
    push_back(a21in);
    push_back(a22in);
    push_back(b2in);
    push_back(c1in);
    push_back(c2in);


}

PCT::PCT( Matches &m, int N, int M, int Method)
{
    _FirstSingVal = -1.0;
    if ( Method == METHOD_EXACT ) {
        Var pDechirp, pRechirp, pFinal;
        double srcCorners[8];
        double dstCorners[8];

        init_var(&pDechirp, "dechirp parameters", 8, 1,
                 INTERNAL_VARIABLE, GREY_SCALE,NULL);
        init_var(&pRechirp, "rechirp parameters", 8, 1,
                 INTERNAL_VARIABLE, GREY_SCALE,NULL);
        init_var(&pFinal, "final parameters", 8, 1,
                 INTERNAL_VARIABLE, GREY_SCALE,NULL);

        for (int i =0; i<8; i++ ) {
            if ( i ==0 || i == 4 ) {
                pDechirp.data[i] = 0;
                pRechirp.data[i] = 0;
                pFinal.data[i] = 0;
            } else {
                pDechirp.data[i] = 0;
                pRechirp.data[i] = 0;
                pFinal.data[i] = 0;
            }
        }

        Matches::iterator it = m.begin();

        srcCorners[0] = (double)it->first->y()/(double)N;
        srcCorners[1] = (double)it->first->x()/(double)M;

        it++;
        srcCorners[2] = (double)it->first->y()/(double)N;
        srcCorners[3] = (double)it->first->x()/(double)M;

        it++;
        srcCorners[4] = (double)it->first->y()/(double)N;
        srcCorners[5] = (double)it->first->x()/(double)M;

        it++;
        srcCorners[6] = (double)it->first->y()/(double)N;
        srcCorners[7] = (double)it->first->x()/(double)M;

        it = m.begin();

        dstCorners[0] = (double)it->second->y() / (double) N;
        dstCorners[1] = (double)it->second->x() / (double) M;

        it++;
        dstCorners[2] = (double)it->second->y() / (double) N;
        dstCorners[3] = (double)it->second->x() / (double) M;

        it++;
        dstCorners[4] = (double)it->second->y() / (double) N;
        dstCorners[5] = (double)it->second->x() / (double) M;

        it++;
        dstCorners[6] = (double)it->second->y() / (double) N;
        dstCorners[7] = (double)it->second->x() / (double) M;

        corners2d(pDechirp.data, dstCorners);
        corners2r(pRechirp.data, srcCorners);

        pcompose(&pDechirp, &pRechirp, &pFinal);
        set(
            pFinal.data[0], pFinal.data[1], pFinal.data[2],
            pFinal.data[3], pFinal.data[4], pFinal.data[5],
            pFinal.data[6], pFinal.data[7] );

        Var_Destructor(&pDechirp);
        Var_Destructor(&pRechirp);
        Var_Destructor(&pFinal);
    }
    else {

        // use an SVD to solve a system of equations.
        //  the model is the PCT,  where x' = PCT(x) and
        //  expanded out and re-arragned like:
        //  a11x + a12y + b1 = c1xx' + c2xy' + x'
        // then put into matrix form:
        // A x =b
        //
        // where A =
        //
        // x y 1 0 0 0 -xx' -x'y
        // 0 0 0 x y 1 -xy' -yy'
        //
        //  repeated for each correspondence
        //
        // x = [ a11 a12 b1...]
        // b = [x' y' ]

        int P,Q;

        P = m.size()*2 ;   // each match gives us 2 rows in the matrix.
        Q = 8;             // 8 parameter PCT

        Array2D< double > A(P,Q) ;
        Array2D< double > SingVal(P,Q) ;      /* create MxN array; all zeros */
        Array2D< double > U(P,P) ;      /* create MxN array; all zeros */
        Array2D< double > V(P,Q) ;      /* create MxN array; all zeros */
        Array2D< double > b(P,1) ;

        Matches::iterator it = m.begin();
        int i=0;
//cout<< "size "<<M<<" by "<<N<<endl;
        for ( it = m.begin() ; it != m.end() ; it++, i+=2 )
        {

//cout <<"match : ("<<it->first->x()<<","<<it->first->y()<<") to (";
//cout <<it->second->x()<<","<<it->second->y()<<endl;

            // fill in the matrix A , normalizing coordinates to [0,1]
            A[i][0] = (double)it->first->x()/(double)N;
            A[i][1] = (double)it->first->y()/(double)M;
            A[i][2] = 1.0;
            A[i][3] = 0.0;
            A[i][4] = 0.0;
            A[i][5] = 0.0;
            A[i][6] = -( (double)it->first->x()*(double)it->second->x() ) /
                      ( (double)N*N );
            A[i][7] = -( (double)it->second->x()*(double)it->first->y() ) /
                      ( (double)N*M );

//for( int j = 0 ; j<8 ; j++ ) { cout <<A[i][j]<<" "; }
//cout<<endl;

            A[i+1][0] = 0.0;
            A[i+1][1] = 0.0;
            A[i+1][2] = 0.0;
            A[i+1][3] = (double)it->first->x()/(double)N;
            A[i+1][4] = (double)it->first->y()/(double)M;
            A[i+1][5] = 1.0;
            A[i+1][6] = -( (double)it->first->x()*(double)it->second->y() ) /
                        ( (double)N*M );
            A[i+1][7] = -( (double)it->first->y()*(double)it->second->y() ) /
                        ( (double)M*M );

//for( int j = 0 ; j<8 ; j++ ) { cout <<A[i+1][j]<<" "; }
//cout<<endl;

            b[i][0] = (double)it->second->x()/(double)N;
            b[i+1][0] = (double)it->second->y()/(double)M;
        }

//for( int j = 0 ; j<m.size()*2 ; j+=2 ) { cout <<b[j][0]<<" "<<b[j+1][0]<<" "; }

        // perform SVD
        SVD<double> x(A);
        x.getS( SingVal );
        x.getU( U );
        x.getV( V );

//cout<<"SINGVALS:"<<endl;
        Array2D< double > NewSingVal(Q,Q) ;
        //grab the diagonal.
        for ( int i=0; i<8 ; i++ )
        {
            for ( int j=0; j<8 ; j++ )
            {
                if ( i==j )
                {
                    assert( SingVal[i][j]!=0.0);
                    NewSingVal[i][j] = 1.0/SingVal[i][j];
                }
                else
                {
                    NewSingVal[i][j] = 0.0;
                }
                // cout<<  NewSingVal[i][j] <<" ";
            }
            //cout << endl;
        }
        // cout << endl;
        //cerr << "SingVal: " <<  SingVal[0][0] <<"  ";
        _FirstSingVal = SingVal[0][0];

        Array2D< double > Utrans(P,P) ;
        for ( int i=0 ; i<P ; i++ )
        {
            for ( int j = 0; j<P ; j++ )
            {
                Utrans[j][i] = U[i][j];
            }
        }
        for ( int i=0 ; i<P ; i++ )
        {
            for ( int j = 0; j<P ; j++ )
            {
                //  cout<<Utrans[i][j]<<" " ;
            }
            //cout<<endl;
        }
        //cout<<endl;

        Array2D< double > params(8,1) ;
        Array2D< double > Utrans_by_b( P,1 ) ;
        Array2D< double > V_by_NewSingVal( 8,8 ) ;

        //zomg why do i have to do this by hand. liek wtf
//cout<<"U by b  = "<<endl;
        assert( Utrans.dim2() == b.dim1() );

        for ( int i=0 ; i<Utrans.dim1() ; i++ ) {
            for ( int j=0 ; j<b.dim2() ; j++ ) {
                Utrans_by_b[i][j] =  0.0;
                for ( int k = 0; k<Utrans.dim2() ; k++ ) {
                    Utrans_by_b[i][j] += Utrans[i][k]*b[k][j];
                }
//cout<<Utrans_by_b[i][j]<<" ";
            }
//cout<<endl;
        }

        for ( int i=0 ; i<V.dim1() ; i++ ) {
            for ( int j=0 ; j<NewSingVal.dim2() ; j++ ) {
                V_by_NewSingVal[i][j] =  0.0;
                for ( int k = 0; k<V.dim2() ; k++ ) {
                    V_by_NewSingVal[i][j] += V[i][k]*NewSingVal[k][j];
                }
            }
        }

        for ( int i=0 ; i<V_by_NewSingVal.dim1() ; i++ ) {
            for ( int j=0 ; j<Utrans_by_b.dim2() ; j++ ) {
                params[i][j] =  0.0;
                for ( int k = 0; k<V_by_NewSingVal.dim2() ; k++ ) {
                    params[i][j] += V_by_NewSingVal[i][k]*Utrans_by_b[k][j];
                }
            }
        }

        //V*NewSingVal*Utrans*b;
        /*
            cerr<<"V is ["<<V.dim1()<<","<<V.dim2()<<"]"<<endl;
            cerr<<"NewSingVal is ["<<NewSingVal.dim1()<<","<<NewSingVal.dim2()<<"]"<<endl;
            cerr<<"Utrans is ["<<Utrans.dim1()<<","<<Utrans.dim2()<<"]"<<endl;
            cerr<<"b ["<<b.dim1()<<","<<b.dim2()<<"]"<<endl;
        */

        /*
            set( params[0][0], params[1][0], params[2][0], params[3][0],
                 params[4][0], params[5][0], params[6][0], params[7][0] );
        */
        set( params[4][0], params[3][0], params[5][0], params[1][0],
             params[0][0], params[2][0], params[7][0], params[6][0] );

    }

}

PCT PCT::operator-(PCT pct2)
{
    PCT newPCT( (a11() - pct2.a11())+1.0,
                a12() - pct2.a12(),
                b1() - pct2.b1(),
                a21() - pct2.a21(),
                (a22() - pct2.a22())+1.0,
                b2() - pct2.b2(),
                c1() - pct2.c1(),
                c2() - pct2.c2() );
    return newPCT;
}

void PCT::operator!()
{
    Var pInitial, pInverted;
    init_var(&pInverted, "inverted parameters", 8, 1,
             INTERNAL_VARIABLE, GREY_SCALE,NULL);
    init_var(&pInitial, "inverted parameters", 8, 1,
             INTERNAL_VARIABLE, GREY_SCALE,NULL);
    for ( int i=0 ; i<8 ;i++ ) pInitial.data[i] = (*this)[i];

    pinverse( &pInitial, &pInverted);


    for ( int i=0 ; i<8 ;i++ ) (*this)[i] = pInverted.data[i];
    /*
     a11 = pInverted.data[0];
     a12 = pInverted.data[1];
     b1  = pInverted.data[2];
     a21 = pInverted.data[3];
     a22 = pInverted.data[4];
     b2  = pInverted.data[5];
     c1  = pInverted.data[6];
     c2  = pInverted.data[7];
    */

    Var_Destructor(&pInitial);
    Var_Destructor(&pInverted);

}



Coords PCT:: project( Coords &p, int W, int H ) {
    double denom = c1()*p.y()/(float)H + c2()*p.x()/(float)W + 1 ;
    double newY = a11()*p.y()/(float)H + a12()*p.x()/(float)W + b1() ;
    double newX = a21()*p.y()/(float)H + a22()*p.x()/(float)W + b2() ;
    return Coords( newX/denom*(double)W, newY/denom*(double)H );
}

double PCT::FirstSingVal()
{
    return _FirstSingVal;
}

