typedef unsigned char uint_8;
typedef signed long int sint_32;//defining this like this becuase i'm not sure if a long int is 4 bytes or not, so can change later

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <cstdlib>
#include <cstdio>
#include "spifunct.h"
#include "compass.h"



int main(void){
//testing correct endianndess

uint_8 array[6] = {0xC2, 0xed, 0x40, 0x00, 0x07, 0x00};
uint_8 outarray[10];
uint_8 * temp;
/*//bytes2sint32LE(&array[0]);
printf("\nLE 0x B4C4, human C4B4 bytes to sint32: %X\n",bytes2sint32LE(&array[0]));

outarray = sint32_bytesLE(500000);
printf("this should be {32,161,7,0): {%u,%u,%u,%u}\n", outarray[0], outarray[1], outarray[2], outarray[3]);


printf("bytes to float:-118.625 %f\n",bytes2floatLE(&array[0]));


outarray = float2bytesLE(-118.625);
printf("0xC2, 0xed, 0x40, 0x00: {%X,%X,%X,%X}\n", outarray[0], outarray[1], outarray[2], outarray[3]);
//compassData foo = CompassDriver.GetData();*/


/*temp = float2bytesLE(-118.625);

outarray[3] = temp[0];
outarray[4] = temp[1];
outarray[5] = temp[2];
outarray[6] = temp[3];

printf("0xC2, 0xed, 0x40, 0x00: {%X,%X,%X,%X}\n", outarray[3], outarray[4], outarray[5], outarray[6]);*/
CalDataResp test;
test.ByteCount = 0xFF;
test.XOffset = 0xFF;
test.YOffset = 0xFF;
test.XGain = 0xFF;
test.YGain = 0xFF;
test.phi = 0xFF;
test.CalibrationMagnitude = 0xFF;

CompassDriver testdriver(7);
//testdriver.SetCalData(test);

compassData testgetdata;
testdriver.SetDataComponents(63);
testgetdata = testdriver.GetData(outarray);

printf("%li",testgetdata.XRaw);

}

