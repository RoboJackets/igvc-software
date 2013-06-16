#ifndef COLORRANGEFINDER_H
#define COLORRANGEFINDER_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

void findRange();
void showMatchingPixels(int bMin, int bMax, int gMin, int gMax, int rMin, int rMax, int reduction=0);
void showMatchingPixels(int bMin, int bMax, int gMin, int gMax, int rMin, int rMax, int reduction, std::string img = "/home/alex/Desktop/BenCode_534_small.jpg");
#endif //COLORRANGEFINDER_H
