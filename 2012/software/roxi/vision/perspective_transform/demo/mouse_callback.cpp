// Implement mouse callback
int ptnum=0;
CvPoint pt[5];//the 5th point never is used

void drawlines();

void mouse_callback( int event, int x, int y, int flags, void* param ){
	flags=flags;//avoid stupid warning
	
	IplImage* image = (IplImage*) param;
	image=image;

	switch( event ){
		case CV_EVENT_MOUSEMOVE: 
			break;

		case CV_EVENT_LBUTTONDOWN:
			break;

		case CV_EVENT_LBUTTONUP:
			pt[ptnum]=cvPoint(x,y);

			if (ptnum<4){//don't write forever
				ptnum++;
			}
			break;
	}
}
