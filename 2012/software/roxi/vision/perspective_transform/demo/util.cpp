#include <iomanip>

//Show an image
void shownow(const char* name,const Mat& m);

// add two points
CvPoint2D32f add(CvPoint2D32f a,CvPoint2D32f b);

//Prints a matrix
void printMat(const CvMat* C);

//Loads a matrix to opengl
void loadMat(const CvMat* C);

//Show an image
void shownow(const char* name,const Mat& m){
	imshow(name,m);
	//waitKey(1);
}

// add two points
CvPoint2D32f add(CvPoint2D32f a,CvPoint2D32f b){
	CvPoint2D32f ret=cvPoint2D32f(a.x+b.x,a.y+b.y);
	return ret;
}

//Prints a matrix
void printMat(const CvMat* C){
cout << setprecision( 5) << right << fixed;
	for ( int row = 0; row < 3; ++ row )
	{
		for ( int col = 0; col < 3; ++ col )
		{
			cout << setw( 5 ) << (double)cvmGet( C, row, col ) << " ";
		}
		cout << endl;
	}
}

//Loads a matrix to opengl
void loadMat(const CvMat* C){
	GLdouble	M[16];

	//upper left part
	for ( int row = 0; row < 2; ++ row )
	{
		for ( int col = 0; col < 2; ++ col )
		{
			M[row*4+col]=(GLdouble)cvmGet( C, row, col );
		}

	}
	
	// lower part
	for(int i=0;i<2;i++)
		M[3+i*4]=(GLdouble)cvmGet( C, 2, i );
	
	// invert y
	for(int i=4;i<=7;i++)
		M[i]=-M[i];
		
	//	corner
	M[15]=(GLdouble)cvmGet( C, 2, 2 );
	glLoadMatrixd(M);
}

//Draws lines on main image
void drawlines(){
	for(int i=1;i<ptnum;i++)if(ptnum>=2){
		cvLine( MainImage, pt[i], pt[i-1], cvScalar(0xff,0x00,0x00), 1, 8, 0);
	}
	if (ptnum>3){
		cvLine( MainImage, pt[3], pt[0], cvScalar(0xff,0x00,0x00), 1, 8, 0);
	}
}
