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

uint_8 char2uint_8(char foo){
	union char2uint8{
		uint_8 uint8;
		char chr;
		}out;
	//out.chr = foo[0];
	out.chr = foo;
	return(out.uint8);
}

char uint_82char2(uint_8 * foo){
	union char2uint8{
		uint_8 uint8;
		char chr;
		}out;
	out.uint8 = foo[0];
	return(out.chr);
}


//uint_8 lastcommand;//for debug

int spiSend(uint_8 * data, int size){
	/*printf("sending:\n");
	for(int i=0; i<size;i++){
		printf("\tpacket %i = %X\n",i,data[i]);
	}
	lastcommand = data[1];*/
	return(0);
}



int spiGet(uint_8 * dataresp, int size){
	//for debug -- depends on commented out global debug var above
	/*if (lastcommand == get_mod_info){
		//uint_8 dataresp[11];
		dataresp[0] = sync_flag;
		dataresp[1] = mod_info_resp;
		dataresp[2] = char2uint_8('V');
		dataresp[3] = char2uint_8('2');
		dataresp[4] = char2uint_8('X');
		dataresp[5] = char2uint_8('e');
		dataresp[6] = char2uint_8('V');
		dataresp[7] = char2uint_8('2');
		dataresp[8] = char2uint_8('0');
		dataresp[9] = char2uint_8('1');
		dataresp[10] = terminator;
	}
	
	if (lastcommand == get_data){
	dataresp[0] = sync_flag;
	dataresp[1] = data_resp;
	dataresp[2] = 2;//compcount
	dataresp[3] = XRaw;

	uint_8 * temp = sint32_bytesLE(500);
		dataresp[4] = temp[0];
		dataresp[5] = temp[1];
		dataresp[6] = temp[2];
		dataresp[7] = temp[3];
	dataresp[8] = YRaw;
	temp = sint32_bytesLE(500);
		dataresp[9] = temp[0];
		dataresp[10] = temp[1];
		dataresp[11] = temp[2];
		dataresp[12] = temp[3];
	dataresp[13] = terminator;

	printf("reciveing:\n");
	}

	if(lastcommand == get_config){
		dataresp[0] = sync_flag;
		dataresp[1] = config_resp;
		dataresp[2] = true_north;
		dataresp[3] = false;
		dataresp[4] = terminator;
		
	}

	if(lastcommand == get_cal_data){
		dataresp[0] = sync_flag;
		dataresp[1] = cal_data_resp;
		dataresp[2] = 24;
			for(int i =3;i<27;i++){
				dataresp[i] = 0;
			}
		dataresp[27] = terminator;
		
	}

	for(int i=0; i<size;i++){
		printf("\tpacket %i = %X\n",i,dataresp[i]);
	}*/
	return(0);
}

