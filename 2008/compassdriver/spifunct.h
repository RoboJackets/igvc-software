

int spiSend(uint_8 * data, int size){
return(0);
}
int spiGet(uint_8 * dataresp, int size){
return(0);
}

sint_32 bytes2sint32LE(uint_8 * array){//give it pointer to least signifiacant byte first (little endian)
	sint_32 foo1 = array[0];//lsb
	sint_32 foo2 = array[1] << 8;
	sint_32 foo3 = array[2] << 16;
	sint_32 foo4 = array[3] << 24;

	return(foo1 + foo2 + foo3 + foo4);
}

uint_8 * sint32_bytesLE(sint_32 input){//give it sint_32, return pointer to byte array, lsb first (little endian)
	/*sint_32 foo1 = input;//lsb
		foo1 = foo1 << 24;
		foo1 = foo1 >> 24;
		uint_8 out1 = foo1;

	sint_32 foo2 = input;
		foo2 = foo2 << 16;
		foo2 = foo2 >>24;
		uint_8 out2 = foo2;

	sint_32 foo3 = input;
		foo3 = foo3 << 8;		
		foo3 = foo3 >> 24;
		uint_8 out3 = foo3;
	
	uint_8 out4 = input >> 24;*/

union sint_32_uint_8{
	sint_32 sint32;
	uint_8 uc[4];
	} inputunion;
	
	inputunion.sint32 = input;

	uint_8 out1 = inputunion.uc[0];
	uint_8 out2 = inputunion.uc[1];
	uint_8 out3 = inputunion.uc[2];
	uint_8 out4 = inputunion.uc[3];

	uint_8 output[4] = {out1, out2, out3, out4};
	return(output);
}

float bytes2float(uint_8 * array){
return(0);
}

uint_8 * float2bytes(float input){//lsb returned first (0=sign; 1-8=exponent;9-31=mantissa)
	/*uint_8 * point = &input;
	uint_8 out1 = point[0];
	uint_8 out2 = point[1];
	uint_8 out3 = point[2];
	uint_8 out4 = point[3];*/
	union float_uint_8{
	float flt;
	uint_8 uc[4];
	} inputunion;

	inputunion.flt = input;
	
	uint_8 out1 = inputunion.uc[0];
	uint_8 out2 = inputunion.uc[1];
	uint_8 out3 = inputunion.uc[2];
	uint_8 out4 = inputunion.uc[3];

/*	float foo1 = input;//lsb
		foo1 = foo1 << 24;
		foo1 = foo1 >> 24;
		uint_8 out1 = foo1;

	float foo2 = input;
		foo2 = foo2 << 16;
		foo2 = foo2 >>24;
		uint_8 out2 = foo2;

	float foo3 = input;
		foo3 = foo3 << 8;		
		foo3 = foo3 >> 24;
		uint_8 out3 = foo3;
	
	uint_8 out4 = input >> 24;*/


uint_8 output[4] = {out1, out2, out3, out4};
return(output);
}
