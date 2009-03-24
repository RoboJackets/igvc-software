#include <math.h>
#include "PointParamEstimator.h"
#include <stdio.h>


// printCv33Matrix is copied from mapgen.cpp
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


/*****************************************************************************/
/*
 */
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

	CvMat* homography = cvCreateMat(3,3,CV_32F);

	CvPoint2D32f p1[2]; // previous pts
	CvPoint2D32f p2[2]; // current pts

	p1[0] = data[0]->first;
	p1[1] = data[0]->second;
	p2[0] = data[1]->first;
	p2[1] = data[1]->second;

	// find homography from pts1 to pts2
	computeHomography(p1,p2,homography);

	// unroll CvMat into a vector for output
	for (int i=0; i<9; i++)
	{
		//parameters.push_back( homography->data.fl[i] );
		parameters.push_back( cvGetReal1D(homography,i) );
	}

	cvReleaseMat( &homography );

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

	CvMat* homography = cvCreateMat(3,3,CV_32F);

	CvPoint2D32f p1[2]; // previous pts
	CvPoint2D32f p2[2]; // current pts

	p1[0] = data[0]->first;
	p1[1] = data[1]->first;
	p2[0] = data[0]->second;
	p2[1] = data[1]->second;

	// find homography from pts1 to pts2
	computeHomography(p1,p2,homography);

	// unroll CvMat into a vector for output
	for (int i=0; i<9; i++)
	{
		//parameters.push_back( homography->data.fl[i] );
		parameters.push_back( cvGetReal1D(homography,i) );
	}

	cvReleaseMat( &homography );

}
/*****************************************************************************/
/*
 */
bool PointParamEstimator::agree(std::vector<double> &parameters, std::pair<CvPoint2D32f ,CvPoint2D32f >  &data)
{

	double error;
	double x1,y1,x2,y2,x2test,y2test;

	// get prev pt
	x1 = data.first.x;
	y1 = data.first.y;

	// get curr pt
	x2 = data.second.x;
	y2 = data.second.y;

	// project a point using input h matrix
	x2test = parameters[0]*x1 + parameters[1]*y1 + parameters[2];
	y2test = parameters[3]*x1 + parameters[4]*y1 + parameters[5];

	// see how close it is to curr point
	error = abs(x2-x2test)+abs(y2-y2test);

	// normalize error
	if (error>0.0) error = 1.0/error;

	//printf("error: %.4f max: %.4f \n",error,m_deltaSquared);
	return ((error) < m_deltaSquared);

}


/*****************************************************************************/
/*
 */
int PointParamEstimator::computeHomography(CvPoint2D32f* prev, CvPoint2D32f* curr, CvMat* h)
{
	/*
	    compute h matrix to map from previous image to current image

	    | cos -sin e |   | x1 |   | x2 |
	    | sin cos  f | * | y1 | = | y2 |
	    | 0   0    1 |   | 1  |   | 1  |

	     prev       curr
	    (x1,y1)<==>(x2,y2)
	    (x3,y3)<==>(x4,y4)
	*/

	double x1,y1,x2,y2,x3,y3,x4,y4,e,f,sint,cost,v1x,v1y,v2x,v2y,delta;

	x1 = prev[0].x;
	y1 = prev[0].y;
	x2 = curr[0].x;
	y2 = curr[0].y;

	x3 = prev[1].x;
	y3 = prev[1].y;
	x4 = curr[1].x;
	y4 = curr[1].y;

	//v1
	v1x = x3-x1;
	v1y = y3-y1;

	//v2
	v2x = x4-x2;
	v2y = y4-y2;

	//delta = |v1xv2|
	delta = v1x*v2y-v2x*v1y ;

	//sintheta = dt / |v1|*|v2|
	sint = delta / ( sqrt(v1x*v1x+v1y*v1y)*sqrt(v2x*v2x+v2y*v2y) );

	//costheta = sqrt ( 1 - sintheta^2 )
	cost = sqrt(1 - (sint)*(sint));

	//translation
	e = x2 - ( cost*x1 + (-sint)*y1 );
	f = y2 - ( (sint)*x1 + cost*y1 );

	//homography
	cvmSet(h,0,0,cost);
	cvmSet(h,0,1,-sint);
	cvmSet(h,0,2, e );
	cvmSet(h,1,0,sint);
	cvmSet(h,1,1, cost);
	cvmSet(h,1,2, f );
	cvmSet(h,2,0, 0  );
	cvmSet(h,2,1, 0   );
	cvmSet(h,2,2, 1 );

	//done
	return 1;

}



