#include <iomanip>

//Connects to firewire camera
int connectToCamera();

//Show an image
void shownow(const char* name,const Mat& m);

// add two points
CvPoint2D32f add(CvPoint2D32f a,CvPoint2D32f b);

//Prints a matrix
void printMat(const CvMat* C);

//Loads a matrix to opengl
void loadMat2d(const CvMat* C);

//Multiplies a matrix to opengl
void multMat2d(const CvMat* Cv);

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
void loadMat2d(const CvMat* C){
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
		
	//	corner
	M[15]=(GLdouble)cvmGet( C, 2, 2 );
	glLoadMatrixd(M);
}

//Multiplies a 2d cvmatrix to opengl
void multMat2d(const CvMat* Cv){

	/* Elements map like so:
	
	0	1	2			0:0	4:1	 8:	12:2
	3	4	5  ->			1:3	5:4	 9:	13:5
	6	7	8			2:	6:	10:1	14:
						3:6	7:7	11:	15:8
	because opencv is row major and gl is col major and we need to fill out missing z components
	*/
	GLdouble	M[16]={0,0,0,0 \
   					,0,0,0,0 \
   					,0,0,0,0 \
   					,0,0,0,0};
   GLdouble	C[9];
   //unpacking matrix to doubles
   for(int i=0;i<9;i++){					
   	C[i]=	cvmGet( Cv, i/3, i%3);	
	}
	
	M[0]=C[0];
	M[1]=C[3];
	M[3]=C[6];
	M[4]=C[1];
	M[5]=C[4];
	M[7]=C[7];
	M[10]=1;//z = identity
	M[12]=C[2];
	M[13]=C[5];
	M[15]=C[8];
	glMultMatrixd(M);
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




