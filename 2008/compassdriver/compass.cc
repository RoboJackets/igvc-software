typedef unsigned char uint_8;
typedef signed long int sint_32;//defining this like this becuase i'm not sure if a long int is 4 bytes or not, so can change later

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "spifunct.h"
//#include "compass.h"
#include <cstdlib>
#include <cstdio>


int main(void){
uint_8 foo[30] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};


printf("\nbytes2sint32LE: %li\n",bytes2sint32LE(&foo[0]));


uint_8 * sint32sint_test1 = sint32_bytesLE(504003);
printf("\nsint32_bytesLE: {%li,%li,%li,%li}\n",sint32sint_test1[0],sint32sint_test1[1],sint32sint_test1[2],sint32sint_test1[3]);


sint_32 sint_test= 504003;
uint_8 * array = sint32_bytesLE(sint_test);
printf("\n%u, %u, %u, %u\n",array[0],array[1],array[2],array[3]);
//printf("\n%li\n",(foo[4])+((foo[3]) << 8)+((foo[2]) << 16)+((foo[1]) << 24));
}

