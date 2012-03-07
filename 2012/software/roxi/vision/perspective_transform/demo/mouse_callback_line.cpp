//THIS FILE TO BE INCLUDED IN linedemo.cpp!!!


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
			cout<<"click"<<endl;
			pt[ptnum]=cvPoint(x,y);
			if (ptnum>0){
				cout<<"click1"<<endl;
				cout<<pt[ptnum].x<<endl;
				cout<<ptnum<<endl;
				//cvLine( image, pt[ptnum], pt[ptnum-1], cvScalar(0xff,0x00,0x00), 1, 8, 0);
				cout<<"click2"<<endl;
			}
			if (ptnum<4){//don't write forever
				ptnum++;
			}
			break;
	}
}
