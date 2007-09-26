

int spiSend(uint_8 * data, int size){
	for(int i=0; i<size;i++){
		printf("packet %i = %X\n",i,data[i]);
	}
	return(0);
}

int spiGet(uint_8 * dataresp, int size){
	for(int i=0; i<size;i++){
		dataresp[i] = 0;
	}
	return(0);
}

sint_32 bytes2sint32LE(uint_8 * array){//give it pointer to least signifiacant byte first (little endian)
	
	union sint_32_uint_8{
	sint_32 sint32;
	uint_8 uint8[4];
	} outputunion;
	
	outputunion.uint8[0] = array[0];//lsb
	outputunion.uint8[1] = array[1];
	outputunion.uint8[2] = array[2];
	outputunion.uint8[3] = array[3];//msb

	return( outputunion.sint32 );
}

uint_8 * sint32_bytesLE(sint_32 input){//give it sint_32, returns pointer to byte array, lsb first (little endian)
	union sint_32_uint_8{
	sint_32 sint32;
	uint_8 uint8[4];
	} inputunion;
	
	inputunion.sint32 = input;

	uint_8 out1 = inputunion.uint8[0];//lsb
	uint_8 out2 = inputunion.uint8[1];
	uint_8 out3 = inputunion.uint8[2];
	uint_8 out4 = inputunion.uint8[3];//msb

	uint_8 output[4] = {out1, out2, out3, out4};
	return(output);
}

float bytes2floatLE(uint_8 * array){
union float_uint_8{
	float flt;
	uint_8 uint8[4];
	} outputunion;
		
	outputunion.uint8[0] = array[3];//haven't checked lsb order, but result seems correct
	outputunion.uint8[1] = array[2];
	outputunion.uint8[2] = array[1];
	outputunion.uint8[3] = array[0];
	
	return(outputunion.flt);
}

uint_8 * float2bytesLE(float input){//lsb returned first (0=sign; 1-8=exponent;9-31=mantissa) -- this works, but don't know about order
	
	union float_uint_8{
	float flt;
	uint_8 uint8[4];
	} inputunion;

	inputunion.flt = input;
	
	uint_8 out1 = inputunion.uint8[0];//lsb??
	uint_8 out2 = inputunion.uint8[1];
	uint_8 out3 = inputunion.uint8[2];
	uint_8 out4 = inputunion.uint8[3];

	uint_8 output[4] = {out1, out2, out3, out4};
	return(output);
}

bool checkbitset(short int foo, int i){//from http://www.cs.umd.edu/class/spring2003/cmsc311/Notes/BitOp/bitI.html
	short int mask = 1 << i;
	return(mask & foo);
}
