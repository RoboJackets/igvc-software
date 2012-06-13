#include "bwareaopen.h"


void clearBorderPixels(IplImage* im){
	for(int x=0; x< im->width; x++){
		im->imageData[x]=0;
	}
	for(int x=0; x< im->width; x++){
		im->imageData[(im->height-1)*im->widthStep+x]=0;
	}
	for(int y=0; y< im->height; y++){
		im->imageData[y*im->widthStep]=0;
	}
	for(int y=0; y< im->height; y++){
		im->imageData[(im->width-1)+y*im->widthStep]=0;
	}
	
}

void bwareaopen(IplImage* im,int area){
	//works like matlab bwareaopen
	clearBorderPixels(im);
	int xstep=im->widthStep;
	char* px = im->imageData;
	int q[(im->height)*(im->width)];//the queue
	int keep[(im->height)*(im->width)];
	int ki=0;
	
	int nbi;
	for(int y = 0;y < im->height;y++){
		for(int x = 0;x < im->width; x++){
			int wpi=y*xstep+x;
			
			
			if (px[wpi]){
				q[0]=wpi;
				int qi=1;
				px[wpi]=0;
				for(int lpn=0; lpn<qi; lpn++){
					int lpi=q[lpn];
					assert(!px[lpi]);
					nbi=lpi-xstep-1;
					if (px[nbi]){
						q[qi++]=nbi;
						px[nbi]=0;
					}
					nbi=lpi-xstep;
					if (px[nbi]){
						q[qi++]=nbi;
						px[nbi]=0;
					}
					nbi=lpi-xstep+1;
					if (px[nbi]){
						q[qi++]=nbi;
						px[nbi]=0;
					}
					nbi=lpi-1;
					if (px[nbi]){
						q[qi++]=nbi;
						px[nbi]=0;
					}
					nbi=lpi+1;
					if (px[nbi]){
						q[qi++]=nbi;
						px[nbi]=0;
					}
					nbi=lpi+xstep-1;
					if (px[nbi]){
						q[qi++]=nbi;
						px[nbi]=0;
					}
					nbi=lpi+xstep;
					if (px[nbi]){
						q[qi++]=nbi;
						px[nbi]=0;
					}
					nbi=lpi+xstep+1;
					if (px[nbi]){
						q[qi++]=nbi;
						px[nbi]=0;
					}
					
				}
				if (qi>=area){
					for(int i=0;i<qi;i++){
						keep[ki++]=q[i];
					}
				}
			}
		}
	}
	for(int i=0;i<ki;i++){
		px[keep[i]]=255;
	}
}
