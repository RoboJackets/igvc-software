#include<stdio.h>
#include "tnt.h"
#include "jama_svd.h"

using namespace TNT;
using namespace JAMA;

//this example program does the same thing that the octave help
// on svd does.  the result should be:
//          ans =
//              1.4083189
//              0.1223271
//              0.0026873

#define M 3
#define N 3
int main()
{
 int i, j;
 Array2D< double > A(M,N) ;      /* create MxN array; all zeros */
 Array2D< double > SingVal(M,N) ;      /* create MxN array; all zeros */

 A[0][0]=  1.00000;
 A[0][1]=  0.50000;
 A[0][2]=  0.33333;
 A[1][0]=  0.50000;  
 A[1][1]=  0.33333;
 A[1][2]=  0.25000;
 A[2][0]=  0.33333;
 A[2][1]=  0.25000;
 A[2][2]=  0.20000;


 SVD<double> x(A);

  x.getS( SingVal);
   for (i=0; i < M; i++) {
      for (j=0; j < N; j++)
	   printf("%g ", SingVal[i][j]);
      printf("\n");
   }   



}


