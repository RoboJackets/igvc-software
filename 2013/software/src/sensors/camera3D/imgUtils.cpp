#include "imgUtils.h"

using namespace std;
using namespace Magick;
void greyscale(Magick::Image& image)
{
    image.quantizeColorSpace( GRAYColorspace );
    image.quantizeColors( 256 );
    image.quantize( );
}

void zeroAllButRed(Magick::Image& image)
{
    int nrows=image.rows();
    int ncols=image.columns();
    char* pixels = (char*)image.getPixels(0,0,ncols,nrows);
    for(int a = 0; a < nrows*ncols*8; a += 8)
    {
        pixels[a] = 0; //b
        pixels[a+1] = 0;
        pixels[a+2] = 0; //g
        pixels[a+3] = 0;


        pixels[a+6] = 0; //a
        pixels[a+7] = 0;
    }
    image.setPixels(0,0,ncols,nrows);
    image.syncPixels();
}


void ZeroAllButGreen(Magick::Image& image)
{
    int nrows=image.rows();
    int ncols=image.columns();
    char* pixels = (char*)image.getPixels(0,0,ncols,nrows);
    for(int a = 0; a < nrows*ncols*8; a += 8)
    {
        pixels[a] = 0; //b
        pixels[a+1] = 0;


        pixels[a+4] = 0; //r
        pixels[a+5] = 0;
        pixels[a+6] = 0; //a
        pixels[a+7] = 0;
    }
    image.setPixels(0,0,ncols,nrows);
    image.syncPixels();
}

void getDispInputImg(string lPath,string rPath, Image& dispInput)
{
    Image lImg;
    Image rImg;
    lImg.read(lPath);
    rImg.read(rPath);
    greyscale(rImg);
    greyscale(lImg);
    zeroAllButRed(lImg);
    ZeroAllButGreen(rImg);
    copyGreen(lImg, rImg);
    dispInput = lImg;
    return;

}

//Images must be the same size
void copyGreen(Image& into, Image& outof)
{
    int nrows=into.rows();
    int ncols=into.columns();
    char* destPix = (char*)into.getPixels(0,0,ncols,nrows);
    char* sourcePix = (char*) outof.getPixels(0,0,ncols,nrows);
    for(int a = 0; a < nrows*ncols*8; a += 8)
    {
        destPix[a+2] = sourcePix[a+2];
        destPix[a+3] = sourcePix[a+3];
    }
    into.setPixels(0,0,ncols,nrows);
    into.syncPixels();
}
