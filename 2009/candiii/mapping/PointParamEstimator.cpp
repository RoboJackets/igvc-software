#include <math.h>
#include "PointParamEstimator.h"
#include <stdio.h>


void printMatrix(CvMat* matrix)
{
    printf("\n");
    int row,col;
    for ( row = 0; row < 3; row++ )
    {
        for ( col = 0; col < 3; col++ )
        {
            printf(" %.2f ",(double)cvmGet( matrix, row, col ));
        }
        printf("\n");
    }
    printf("\n");
}


PointParamEstimator::PointParamEstimator(double delta) : m_deltaSquared(delta*delta) {}
/*****************************************************************************/
/*
 */
void PointParamEstimator::estimate(std::vector< std::pair<CvPoint2D32f ,CvPoint2D32f >* > &data,
                                  std::vector<double> &parameters)
{
    parameters.clear();
    if (data.size()<2)
        return;

	{
		CvMat* homography = cvCreateMat(3,3,CV_32F);

		CvPoint2D32f p1[2];
		CvPoint2D32f p2[2];
		p1[0] = data[0]->first;
		p1[1] = data[0]->second;
		p2[0] = data[1]->first;
		p2[1] = data[1]->second;

		computeHomography(p1,p2,homography);

		for (int i=0; i<9; i++){
//			parameters.push_back( homography->data.fl[i] );
			parameters.push_back( cvGetReal1D(homography,i) );
		}

		cvReleaseMat( &homography );
	}




}
/*****************************************************************************/
/*
 */
void PointParamEstimator::leastSquaresEstimate(std::vector< std::pair<CvPoint2D32f ,CvPoint2D32f >* > &data,
        std::vector<double> &parameters)
{
    parameters.clear();
    if (data.size()<2)
        return;


	{
		CvMat* homography = cvCreateMat(3,3,CV_32F);

		CvPoint2D32f p1[2];
		CvPoint2D32f p2[2];
		p1[0] = data[0]->first;
		p1[1] = data[1]->first;
		p2[0] = data[0]->second;
		p2[1] = data[1]->second;

		computeHomography(p1,p2,homography);

		for (int i=0; i<9; i++){
//			parameters.push_back( homography->data.fl[i] );
			parameters.push_back( cvGetReal1D(homography,i) );
		}

		cvReleaseMat( &homography );
	}


}
/*****************************************************************************/
/*

 */
bool PointParamEstimator::agree(std::vector<double> &parameters, std::pair<CvPoint2D32f ,CvPoint2D32f >  &data)
{

	double error;

	{
		double x1,y1,x2,y2,x2test,y2test;

		x1 = data.first.x;
		y1 = data.first.y;
		x2 = data.second.x;
		y2 = data.second.y;

		x2test = parameters[0]*x1 + parameters[1]*y1 + parameters[2];
		y2test = parameters[3]*x1 + parameters[4]*y1 + parameters[5];

//		error =  (1.0/abs(x2-x2test)+1.0/abs(y2-y2test));
//		error =  1.0/((x2-x2test)*(x2-x2test))+1.0/((y2-y2test)*(y2-y2test));
		error =  (abs(x2-x2test)+abs(y2-y2test));
		if(error>0.0) error = 1.0/error;

		//printf("E  1(%.2f,%.2f) 2(%.2f %.2f)   \n" , x1,y1,x2,y2  );
	}


//	printf("error: %.4f max: %.4f \n",error*error,m_deltaSquared);
//    return ((error*error) < m_deltaSquared);
	//printf("error: %.4f max: %.4f \n",error,m_deltaSquared);
    return ((error) < m_deltaSquared);
}




/*****************************************************************************/

int PointParamEstimator::computeHomography(CvPoint2D32f* p1, CvPoint2D32f* p2, CvMat* h)
{
    /*
        | 1 b e |   | x1 |   | x3 |
        | c 1 f | * | y1 | = | y3 |
        | 0 0 1 |   | 1  |   | 1  |

        (x1,y1)<==>(x3,y3) && (x2,y2)<==>(x4,y4)
    */

    double x1,y1,x2,y2,x3,y3,x4,y4,b,c,e,f;

    x1 = p1[0].x;
    y1 = p1[0].y;
    x2 = p2[0].x;
    y2 = p2[0].y;

    x3 = p1[1].x;
    y3 = p1[1].y;
    x4 = p2[1].x;
    y4 = p2[1].y;

    b = ( x2-x4-x1+x3 ) / ( y1-y3 ) ;
    c = ( y2-y4-y1+y3 ) / ( x1-x3 ) ;

    //e = ( x2+x4-x1-x3-b*(y1+y3) ) / 2 ;
    //f = ( y2+y4-y1-y3-c*(x1+x3) ) / 2 ;
    e = x2-x1-b*y1;
    f = y2-c*x1-y1;

    cvZero(h);

    cvmSet(h,0,0, 1 ); cvmSet(h,0,1, b ); cvmSet(h,0,2, e );
    cvmSet(h,1,0, c ); cvmSet(h,1,1, 1 ); cvmSet(h,1,2, f );
    cvmSet(h,2,0, 0 ); cvmSet(h,2,1, 0 ); cvmSet(h,2,2, 1 );

    //printf(" 1(%.2f,%.2f) 2(%.2f %.2f) | 3(%.2f,%.2f) 4(%.2f %.2f)  \n" , x1,y1,x2,y2,x3,y3,x4,y4 );
    //printf(" b%.2f c%.2f e%.2f f%.2f  \n" , b,c,e,f );
	//printMatrix(h);

    if( y1==y3 || x1==x3 ) return 0;
    else return 1;

}



