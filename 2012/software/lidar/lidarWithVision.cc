#include "lidarWithVision.hpp"

const int RADIUS=3;

int height;
int width;




IplImage correctVisionWithLidar(float lidarX[], float lidarY[], IplImage* visionImage){
	height=visionImage->height;
	width=visionImage->width;
	int channels=visionImage->nChannels;
	int imageArraySize=width*height;
	bool lidarBitMap[height*width];
	std::fill_n(lidarBitMap,sizeof(lidarBitMap)/sizeof(bool),false);
//Construct blotted pixel representation of lidar places
	for(int i=0;0<sizeof(lidarX)/sizeof(float);i++){
		for(int y=(int)lidarY[i]-RADIUS/2;y<(int)lidarY[i]+RADIUS;y++){
			for(int x=(int)lidarX[i]-RADIUS/2;x<(int)lidarX[i]+RADIUS;x++){
					if((y*width+x)<imageArraySize && (y*width+x)>0)
						lidarBitMap[y*width+x]=true;
				}
			}
        }
		

	
	//Construct checked array with all starting of as unchecked
	for(int i=0;i<sizeof(lidarBitMap)/sizeof(bool);i++){
                if(lidarBitMap[i]){
                	bool checked[imageArraySize];
                	std::fill_n(checked,sizeof(checked)/sizeof(bool),false);
                        Shape presentShape;
                        buildShape(i%width,i/width,checked,lidarBitMap,&presentShape);
                        eraseShapeOnBitmap(presentShape,lidarBitMap);
                        eraseShadow(presentShape, &visionImage);
	}
	}	
}


void buildShape(int currX,int currY, bool* checked, bool* bitMap,std::vector<Point>* currShape){
        Point currPoint={currX,currY};
	checked[currX+currY*width]=true;
        
	if(bitMap[currX+currY*width]){
            addPointToShape(currPoint,currShape);
		if(!checked[(currX+1)+currY*width]){
                    buildShape(currX+1,currY, checked, bitMap,currShape);
		}
		if(!checked[currX+(currY+1)*width]){
                    buildShape(currX,currY+1, checked, bitMap,currShape);
		}
		if(!checked[(currX+1)+(currY+1)*width]){
                    buildShape(currX+1,currY+1, checked, bitMap,currShape);
		}
		if(!checked[(currX-1)+(currY)*width]){
                    buildShape(currX-1,currY, checked, bitMap,currShape);
		}
                 if(!checked[currX+(currY-1)*width]){
                     buildShape(currX,currY-1, checked, bitMap,currShape);
		}
		if(!checked[(currX-1)+(currY-1)*width]){
                    buildShape(currX-1,currY-1, checked, bitMap,currShape);
		}
		if(!checked[(currX+1)+(currY-1)*width]){
                     buildShape(currX+1,currY-1, checked, bitMap,currShape);
		}
		if(!checked[(currX-1)+(currY+1)*width]){
                     buildShape(currX-1,currY+1, checked, bitMap,currShape);
		}	
	}
}

void addPointToShape(Point toAdd, Shape* addTo){
	addTo->points.push_back(toAdd);
	addTo->totalX+=toAdd.x;
	addTo->totalY+=toAdd.y;
	if(toAdd.x>addTo->maxX)
		addTo->maxX=toAdd.x;
	if(toAdd.y>addTo->maxY)
		addTo->maxY=toAdd.y;
	if(toAdd.x<addTo->minX)
		addTo->minX=toAdd.x;
	if(toAdd.y<addTo->minY)
		addTo->minY=toAdd.y;
}

void eraseShapeOnBitmap(Shape shape,bool** eraseFrom){
    for(int i=0;i<shape.numPoints;i++){
        (*eraseFrom)[shape.points.at(i).x+shape.points.at(i).y*width]=false;
    }
}

void eraseShadow(Shape shape, IplImage* img){


}


