#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "SDL/SDL.h"
#include "SDL/SDL_image.h"
#include <SDL/SDL_ttf.h>
#include <cmath>
#include <iostream>
#include <sstream>
#define axisInvert -1 // 1 for inverted y analog axes , -1 for non-inverted
#define PI 3.14159

//Global Motion Control Variables

    //Used by Single Stick Drive Mode
    extern float  Speed;
    extern float  Heading;
    //Used by Dual Stick Drive Mode
    extern int leftSpeed, rightSpeed;
    extern int leftDirection, rightDirection;




//The Joystick
class Joystick
{
    public:
    bool initJoystick(); //Initialize Joystick
    void readJoystick();
    void setMotion();
    void cleanJoystick();
    bool Quit();
    //Joystick Buttons
    int leftAnalogX, leftAnalogY;
    int rightAnalogX, rightAnalogY;

    private:
    //The Joystick SDL Object
    SDL_Joystick *joystick;

    int dPadX, dPadY, dPadA, dPadB;
    int DigitalUp, DigitalLeft, DigitalRight, DigitalDown;
    int SENSITIVITY; //Joystick Sensitivity
};



//The Display Graph
class Graph
{
    public:
    enum {
    //Screen attributes
    SCREEN_WIDTH = 640, SCREEN_HEIGHT = 480, SCREEN_BPP = 32,
    //The frame rate
    FRAMES_PER_SECOND = 20,
    //The Speed Label dimensions
    LABEL_WIDTH = 20, LABEL_HEIGHT = 20,
    //The Compass dimensions
    COMPASS_WIDTH = 20, COMPASS_HEIGHT = 20,
    //The Speed # dimensions
    SPEED_WIDTH = 20, SPEED_HEIGHT = 20,
     //The Arrow dimensions
    ARROW_WIDTH = 20, ARROW_HEIGHT = 20
    };
    //Initialize Screen
    bool initGraph();
    //Load Images
    bool loadGraph();
    //Display Graph
    void displayGraph();
    //Apply Images
    void applyImage(int x, int y, SDL_Surface* source, SDL_Surface* destination, SDL_Rect* clip);
    //Apply Texts
    void applyText(int x, int y, double s, SDL_Surface* destination, SDL_Rect* clip);
    //Clean and close Graph
    void cleanGraph();
    bool Quit();


    private:
    SDL_Surface *compass;
    SDL_Surface *screen;
    SDL_Surface *leftspeedlabel;
    SDL_Surface *rightspeedlabel;
    SDL_Surface *gaugeframe;
    SDL_Surface *gaugebar;
    SDL_Surface *arrow;

    SDL_Surface *leftspeedtext;
    SDL_Surface *rightspeedtext;
    TTF_Font *font;

    SDL_Surface* source;
    SDL_Surface* destination;
    SDL_Rect* clip;
    //Graph Objects' Screen Offsets:
    //The offsets of the Compass
    int xCompass, yCompass;
    //The offsets of the Left Gauge frame
    int xleftGaugeframe, yleftGaugeframe;
    //The offsets of the Right Gauge frame
    int xrightGaugeframe, yrightGaugeframe;
    //The offsets of the left Gauge bar
    int xleftGaugebar, yleftGaugebar;
    //The offsets of the Right Gauge bar
    int xrightGaugebar, yrightGaugebar;
    //The offsets of the Left Speed Label
    int xleftLabel, yleftLabel;
    //The offsets of the Right Speed Label
    int xrightLabel, yrightLabel;
    //The offsets of the left Speed#
    int xleftSpeed, yleftSpeed;
    //The offsets of the right Speed#
    int xrightSpeed, yrightSpeed;
    //The offsets of the Arrow Center
    int xArrow, yArrow;
    double thArrow;

};


//The Timer
class Timer
{
    private:
    //The clock time when the timer started
    int startTicks;
    //The ticks stored when the timer was paused
    int pausedTicks;
    //The timer status
    bool paused;
    bool started;

    public:
    //Initializes variables
    bool initTimer();
    //The various clock actions
    void start();
    void stop();
    void pause();
    void unpause();
    //Gets the timer's time
    int get_ticks();
    //Checks the status of the timer
    bool is_started();
    bool is_paused();

};

#endif // JOYSTICK_H
