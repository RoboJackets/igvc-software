#ifndef IMGUTILS_H
#define IMGUTILS_H

#include <fstream>
#include <iostream>
#include <string>
#include <Magick++.h>

using namespace std;

void greyscale(Magick::Image& image);
void zeroAllButRed(Magick::Image& image);
void ZeroAllButGreen(Magick::Image& image);
void copyGreen(Magick::Image& into, Magick::Image& outof);
void getDispInputImg(string lPath,string rPath, Magick::Image& dispInput);
#endif // IMGUTILS_H
