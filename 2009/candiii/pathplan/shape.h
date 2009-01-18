#include <math.h>
#include <string>
#include <iostream>      //Included for ostream class (cout)

#define PI 3.14159
using namespace std;

class pixel
{

public:
	int x,y;
	pixel()
	{
		x=0;
		y=0;
	}
	pixel(int x, int y)
	{
		this->x = x;
		this->y = y;
	}
};


class shape
{

public:
	int x,y;
	pixel **outline;
	int size;
	int loc,stepX,stepY;

public:
	shape()
	{
		x = 0;
		y = 0;
	}
	void setLocation(int x, int y)
	{
		this->x = x;
		this->y = y;
	}
	int isValidLocation(unsigned char *image, int xsize, int ysize)
	{

		for (int i =0 ; i < size; i++)
		{
			stepX = this->x + (outline[i])->x;
			stepY = this->y + (outline[i])->y;

			//			if (stepX < 0 || stepX >= xsize){ return 0; }
			//			if (stepY < 0 || stepY >= ysize){ return 0; }

			if (stepX < 0 )
				stepX=0;
			if (stepX >= xsize)
				stepX=xsize-1;
			if (stepY < 0 )
				stepY=0;
			if (stepY >= ysize)
				stepY=ysize-1;

			loc = stepY*xsize + stepX;

			if (image[loc]==0)
			{
				return 0;
			}
		}
		return 1;
	}
	void drawOutline(unsigned char *image, int xsize)
	{

		for (int i =0 ; i < size; i++)
		{
			loc = (int)rint(this->y + (outline[i])->y)*xsize + rint(this->x + (outline[i])->x);
			image[loc]=80;
		}
	}

	void addtoOutline(float stepX,float stepY,float length,float phi, pixel**& outline,int &counter)
	{

		float N = length*5,x_increment,y_increment;
		int addX,addY,prevX,prevY;

		x_increment = length*cos(radians(phi))/N;
		y_increment = length*sin(radians(phi))/N;

		stepX = stepX + x_increment;
		stepY = stepY + y_increment;

		prevX = (int)stepX;
		prevY = (int)stepY;
		outline[counter] = new pixel(prevX,prevY);
		counter++;
		for (int i=0; i<N-1 ; i++)
		{
			stepX = stepX + x_increment;
			stepY = stepY + y_increment;
			addX = (int)stepX;
			addY = (int)stepY;
			if (!((addX==prevX)&&(addY==prevY)))
			{
				outline[counter] = new pixel(addX,addY);
				prevX = addX;
				prevY = addY;
				counter++;
			}
		}
	}

	inline float rint(float x)
	{
		return float((int)(x+.5));
	}
	inline float radians(float x)
	{
		return x*(float)(PI/180);
	}
};

class rectangle : public shape
{

public:
	float width;
	float height;

public:
	rectangle()
	{
		width = 0;
		height = 0;
	}

	void setParams(float width,float height)
	{
		float stepX, stepY,length,phi;
		int counter;
		outline = new pixel*[(int)width*(int)height*50];

		counter = 0;
		stepX = -rint(width/2);
		stepY = -rint(height/2);
		length = width;
		phi = 0;

		addtoOutline(stepX,stepY,length,phi,outline,counter);

		stepX += width*cos(radians(phi));
		stepY += width*sin(radians(phi));
		length = height;
		phi+=90;

		addtoOutline(stepX,stepY,length,phi,outline,counter);

		stepX += height*cos(radians(phi));
		stepY += height*sin(radians(phi));
		length = width;
		phi+=90;

		addtoOutline(stepX,stepY,length,phi,outline,counter);

		stepX += width*cos(radians(phi));
		stepY += width*sin(radians(phi));
		length = height;
		phi+=90;

		addtoOutline(stepX,stepY,length,phi,outline,counter);

		size = counter;
	}
};

