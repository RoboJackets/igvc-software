#ifndef PINDEFS_HPP_
#define PINDEFS_HPP_

const int leftCurrentADCPin	= 3;
const int rightCurrentADCPin	= 4;

/* PIN DEFINITIONS */
/** SPI **/
//#define SPI_MOSI 				/*13*/
const int SPI_MISO			= 7;/*12*/
//const int SPI_CLK			= 6;/*11*/
const int SPI_CLK			= 13;/*11*/
const int SPI_SS_LEFT_MOTOR_ENCODER	= 5;/*10*/
const int SPI_SS_RIGHT_MOTOR_ENCODER	= 4;/*9*/

/* Pin Definitions */
const int rightDirectionPin = 2;
const int rightSpeedPin = 3;
const int rightDisablePin = 4;
const int leftDirectionPin = 5;
const int leftSpeedPin = 6;
const int leftDisablePin = 7;

const int joystickXADC = 0;
const int joystickYADC = 1;
const int joystickEnable = 8;

const int LIGHT_PIN_LB = 13;

/*
const int rightDirectionPin = 4;
const int rightSpeedPin = 3;
const int rightDisablePin = 2;
const int leftDirectionPin = 7;
const int leftSpeedPin = 6;
const int leftDisablePin = 5;
*/
#endif
