
#include <cstdlib>
#include <cstdio>
//#include "spifunct.h.debug"
#include "compass.h"



int main(void){
//testing correct endianndess

uint_8 array[6] = {0xC2, 0xed, 0x40, 0x00, 0x07, 0x00};
uint_8 outarray[10];
uint_8 * temp;

CalDataResp testcaldata;
testcaldata.ByteCount = 24;
testcaldata.XOffset = 0xFF;
testcaldata.YOffset = 0xFF;
testcaldata.XGain = 0xFF;
testcaldata.YGain = 0xFF;
testcaldata.phi = 0xFF;
testcaldata.CalibrationMagnitude = 0xFF;

CompassDriver testdriver(7);
testdriver.GetModInfo();//works
testdriver.SetDataComponents(3);//works

/*compassData testgetdata;
testgetdata = testdriver.GetData();//seems to work
printf("\n\nXRaw: %li",testgetdata.XRaw);
printf("\n\nYRaw: %li",testgetdata.YRaw);*/

/*testdriver.SetConfig(declination, float2bytesLE(25));//seems to work
uint_8 test[1]= {false};
testdriver.SetConfig(true_north, test);*/

//testdriver.GetConfig(true_north);//works

//testdriver.SaveConfig();//works
//testdriver.StartCal();
//testdriver.StopCal();

//testcaldata = testdriver.GetCalData();
//printf("%i",test.YOffset);

//testdriver.SetCalData(testcaldata);//works
}

