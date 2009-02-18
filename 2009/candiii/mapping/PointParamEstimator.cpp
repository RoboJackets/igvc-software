#include <math.h>
#include "PointParamEstimator.h"
#include <stdio.h>


void printCv33Matrix(CvMat* matrix)
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


PointParamEstimator::PointParamEstimator(double delta) : m_deltaSquared(delta*delta)
{
}

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
	double x1,y1,x2,y2,x2test,y2test;

	x1 = data.first.x;
	y1 = data.first.y;
	x2 = data.second.x;
	y2 = data.second.y;

	x2test = parameters[0]*x1 + parameters[1]*y1 + parameters[2];
	y2test = parameters[3]*x1 + parameters[4]*y1 + parameters[5];

//	error =  (1.0/abs(x2-x2test)+1.0/abs(y2-y2test));
//	error =  1.0/((x2-x2test)*(x2-x2test))+1.0/((y2-y2test)*(y2-y2test));
	error =  (abs(x2-x2test)+abs(y2-y2test));
	if(error>0.0) error = 1.0/error;

//	printf("E  1(%.2f,%.2f) 2(%.2f %.2f)   \n" , x1,y1,x2,y2  );
//	printf("error: %.4f max: %.4f \n",error*error,m_deltaSquared);
//  return ((error*error) < m_deltaSquared);
//	printf("error: %.4f max: %.4f \n",error,m_deltaSquared);
    return ((error) < m_deltaSquared);

}


/*****************************************************************************/
/*
 */
int PointParamEstimator::computeHomography(CvPoint2D32f* p1, CvPoint2D32f* p2, CvMat* h)
{
    /*
        | cos -sin e |   | x1 |   | x2 |
        | sin cos  f | * | y1 | = | y2 |
        | 0   0    1 |   | 1  |   | 1  |

         p1         p2
        (x1,y1)<==>(x2,y2)
        (x3,y3)<==>(x4,y4)
    */

    double x1,y1,x2,y2,x3,y3,x4,y4,e,f,sint,cost,v1x,v1y,v2x,v2y,dt;

    //v1
    x1 = p1[0].x;
    y1 = p1[0].y;
    x2 = p2[0].x;
    y2 = p2[0].y;

    //v2
    x3 = p1[1].x;
    y3 = p1[1].y;
    x4 = p2[1].x;
    y4 = p2[1].y;

    //v1
    v1x = x3-x1;
    v1y = y3-y1;

    //v2
    v2x = x4-x2;
    v2y = y4-y2;

    //dt = |v1xv2|
    //dt = sqrt( (v1x*v2y-v2x*v1y)*(v1x*v2y-v2x*v1y) );
    dt = sqrt( (v2x*v1y-v1x*v2y)*(v2x*v1y-v1x*v2y) );

    //sintheta = dt / |v1|*|v2|
    //sint = dt / ( sqrt(v1x*v1x+v1y*v1y)*sqrt(v2x*v2x+v2y*v2y) );
    sint = dt / ( sqrt(v2x*v2x+v2y*v2y)*sqrt(v1x*v1x+v1y*v1y) );

    //costheta = 1 - sintheta^2
    cost = 1 - (sint)*(sint);

    //translation
    e = v2x-v1x;
    f = v2y-v1y;
//    e = (x2+x4-x1-x3)/2;
//    f = (y2+y4-y1-y3)/2;


    //homography
    cvmSet(h,0,0,cost); cvmSet(h,0,1,sint); cvmSet(h,0,2, e );
    cvmSet(h,1,0,-sint); cvmSet(h,1,1,cost); cvmSet(h,1,2, f );
    cvmSet(h,2,0, 0   ); cvmSet(h,2,1, 0   ); cvmSet(h,2,2, 1 );

    //done
    return 1;

}



