#include "lidarWithVision.hpp"

const int RADIUS=3;

struct point{
	int x;
	int y;
};

int height;
int width;




IplImage correctVisionWithLidar(float lidarX[], float lidarY[], IplImage* visionImage){
	height=visionImage->height;
	width=visionImage->width;
	int channels=visionImage->nChannels;
	int imageArraySize=width*height;
	bool lidarBitMap[height*weight];
	std:fill_n(lidarBitMap,sizeof(lidarBitMap)/sizeof(bool),false);
//Construct blotted pixel representation of lidar places
	for(int i=0;0<sizeof(lidarX)/sizeof(float);i++){
		for(int y=(int)lidarY[i]-RADIUS/2;y<(int)lidarY[i]+RADIUS;y++){
			for(int x=(int)lidarX[i]-RADIUS/2;x<(int)lidarX[i]+RADIUS;x++){
					if((y*width+x)<imageArraySize && (y*width+x)>0)
						lidarBitMap[y*width+x]=true;
				}
			}	
		}
	}
	//Construct checked array with all starting of as unchecked
	
	for(int i=0;i<sizeof(lidarBitMap)/sizeof(bool),i++){
	if(lidarBitMap[i]){
		bool checked[imageArraySize];
		std:fill_n(checked,sizeof(checked)/sizeof(bool),false);
		std::vector <Point> shape;
		buildShape(i%width,i/width,checked,&shape);
		eraseShadow(shape, &visionImage);

	}
	}
	
}


void buildShape(int currX,int currY, bool* checked, vector<Point>* currShape){


}

void eraseShapeOnBitmap(vector<Point> shape,bool** eraseFrom){

}

void eraseShadow(vector<Point> shape, IplImage* eraseFrom){


}

