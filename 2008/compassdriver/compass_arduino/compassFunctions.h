compassSend(uint_8 * data, int size) {

	uint_8 senddata[size+2];
	senddata[0] = sync_flag;
	senddata[size+1] = terminator;
	for(int i=0;i<size; i++) {
		senddata[i+1] = data[i];
	}
	spiSend(senddata, size+2);//send data wrapped with sync_flag and terminator
	return(0);
}


void spiSend(uint_8 * dataout, int length){
	for(int frame = 0; frame < length; frame++){
		for(int bit = 0; bit < 8; bit++){
			digitalWrite(SPI_CLK, HIGH);
			if((command[frame] >> bit) & 1){//this is sending little endian -- lsb first
				digitalWrite(SPI_MOSI, HIGH);
			}
			else{
				digitalWrite(SPI_MOSI, LOW);
			}
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
			digitalWrite(SPI_CLK, LOW);
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
		}
		//shiftOut(SPI_MOSI, SPI_CLK, LSBFIRST, command[frame]);//how fast does this run?
	}
}

void compassGet(uint_8 * datain, int length){
	digitalWrite(SPI_MOSI, LOW);//transmit zero whilst receiving
	for(int frame = 0; frame < length; frame++){
		for(int bit = 0; bit < 8; bit++){//this is rec little endian
			digitalWrite(SPI_CLK, HIGH);
			//delayMicroseconds(10);//delay of 10us before data is sent

			datatemp[frame] |= digitalRead(SPI_MISO);
			datatemp[frame] <<= 1;
			
			digitalWrite(SPI_CLK, LOW);
			delayMicroseconds(50);//wait the appropriate amount to make this a 2khz signal (or shorter, max 3.6864 Mhz)
		}
	}
}


void getmodinfo(){
	uint_8 command = get_mod_info;
	uint_8 answer[11];

	compassSend(&command, 1);

	compassGet(answer, 11);

	for(int i = 2; i < 6; i++){
		Serial.print(answer[i], BYTE);
	}
		Serial.print('\n');
	for(int i = 6; i < 10; i++){
		Serial.print(answer[i], BYTE);
	}
	
	//return();
}

//how should this send the data, just print over serial instead of returning struct?
void getData(void) {
	uint_8 data = get_data;
	compassSend(&data,1);

	uint_8 dataresp[DATA_LENGTH + 4];//we will be getting DATA_LENGTH + sync + data_resp + count + term
	spidriver.spiGet(dataresp, DATA_LENGTH + 4);
		

	compassData datarespstruct;
	int i = 3;//first data type packet is byte 4 (3 in array)

	
	float_u8 tempflt;
	s32_u8 temps32;

	if(dataresp[2] == DATA_FRAME_COUNT) {//check for the number of bytes we expect to get
		for(int j = 0; j < DATA_FRAME_COUNT; j++) {

			if(dataresp[i] == XRaw) {
				temps32.u8[0] = dataresp[i+1];
				temps32.u8[1] = dataresp[i+2];
				temps32.u8[2] = dataresp[i+3];
				temps32.u8[3] = dataresp[i+4];
				datarespstruct.XRaw = temps32.s32;
				i += 5;//go forward 5 bytes to next data header
			}
			else if(dataresp[i] == YRaw) {
				temps32.u8[0] = dataresp[i+1];
				temps32.u8[1] = dataresp[i+2];
				temps32.u8[2] = dataresp[i+3];
				temps32.u8[3] = dataresp[i+4];
				datarespstruct.YRaw = temps32.s32;
				i += 5;
			}
			else if(dataresp[i] == XCal) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.XCal = tempflt.flt;
				i += 5;
			}
			else if(dataresp[i] == YCal) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.YCal = tempflt.flt;
				i += 5;
			}
			else if(dataresp[i] == Heading) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.Heading = tempflt.flt;
				i += 5;
			}
			else if(dataresp[i] == Magnitude) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.Magnitude = tempflt.flt;
				i +=5;
			}
			else if(dataresp[i] == Temperature) {
				tempflt.u8[0] = dataresp[i+1];
				tempflt.u8[1] = dataresp[i+2];
				tempflt.u8[2] = dataresp[i+3];
				tempflt.u8[3] = dataresp[i+4];
				datarespstruct.Temperature = tempflt.flt;
				i += 5;
			}
			else if(dataresp[i] == Distortion) {
				datarespstruct.Distortion = dataresp[i+1];//says bool, but guessing send bytes at a time
				i += 2;
			}
			else if(dataresp[i] == CalStatus) {
				datarespstruct.CalStatus = dataresp[i+1];
				i += 2;
			}
			else {
				//handle the error somehow (data type bit didn't match)
				//printf("unknown data type sent\n");
				return(datarespstruct);
			}
		}//end for loop

		return(datarespstruct);

	}//end if block
	else {//we didn't recieve the right number of bytes
		//printf("wrong number of bytes sent");
		return(datarespstruct);
	}
}



int CompassDriver::SetConfig(uint_8 config_id, uint_8 * configval) {
	
	if (config_id == declination) {
		uint_8 data[6];
		data[0] = set_config;
		data[1] = config_id;
			data[2] = configval[0];
			data[3] = configval[1];
			data[4] = configval[2];
			data[5] = configval[3];
		compassSend(data,6);
	}
	else {
		uint_8 data[3];
		data[0] = set_config;
		data[1] = config_id;
		data[3] = configval[0];
		compassSend(data,3);
	}

	return(0);
}


int CompassDriver::StartCal(void) {
	uint_8 data = start_cal;
	compassSend(&data,1);
	return(0);//something about wanting to read XRaw and YRaw??? needs to be set before this is run

}

int CompassDriver::StopCal(void) {
	uint_8 data = stop_cal;
	compassSend(&data,1);
	return(0);//need to SaveConfig for this to be permanent
}

CalDataResp CompassDriver::GetCalData(void) {
	uint_8 data = get_cal_data;
	compassSend(&data,1);
	CalDataResp reply;
	uint_8 resp[28];//is this right? needs to be total length of cal data from device.  (4 x sint_32, 2 x float32) = 24 bytes, + count + sync + term + cal_data_resp
	spidriver.spiGet(resp, 28);//getting array pf bytes from compass


	float_u8 tempflt;
	s32_u8 temps32;
	//sorting and converting bytes
	//if(resp[2] == 24) {//can add this to see if header is correct
		temps32.u8[0] = resp[3];
		temps32.u8[1] = resp[4];
		temps32.u8[2] = resp[5];
		temps32.u8[3] = resp[6];
		reply.XOffset = temps32.s32;

		temps32.u8[0] = resp[7];
		temps32.u8[1] = resp[8];
		temps32.u8[2] = resp[9];
		temps32.u8[3] = resp[10];
		reply.YOffset = temps32.s32;

		temps32.u8[0] = resp[11];
		temps32.u8[1] = resp[12];
		temps32.u8[2] = resp[13];
		temps32.u8[3] = resp[14];
		reply.XGain = temps32.s32;

		temps32.u8[0] = resp[15];
		temps32.u8[1] = resp[16];
		temps32.u8[2] = resp[17];
		temps32.u8[3] = resp[18];
		reply.YGain = temps32.s32;

		tempflt.u8[0] = resp[19];
		tempflt.u8[1] = resp[20];
		tempflt.u8[2] = resp[21];
		tempflt.u8[3] = resp[22];
		reply.phi = tempflt.flt;

		tempflt.u8[0] = resp[23];
		tempflt.u8[1] = resp[24];
		tempflt.u8[2] = resp[25];
		tempflt.u8[3] = resp[26];
		reply.CalibrationMagnitude = tempflt.flt;

	//}
	//else{
		//handle error
		//return(error);
	//}
	
	return(reply);//return structure
}


int CompassDriver::SetDataComponents(DataTypeReq datatypewanted) {
	uint_8 data[11];//max size of config + config count + command, excluding header (compassSend adds it).  the number of bits sent is determind by j, so the unused bits should be ignored
	
	CompassDriver::datapackcount = 0;
	CompassDriver::datalength = 0;
	int j = 2;

	if(datatypewanted.xraw){
		data[j] = XRaw;
		j++;
		datapackcount += 1;//number of data types expected
		datalength += (4 + 1);//length of data + byte label
	}
	if(datatypewanted.yraw){
		data[j] = YRaw;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.xcal){
		data[j] = XCal;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.ycal){
		data[j] = YCal;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.heading){
		data[j] = Heading;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.magnitude){
		data[j] = Magnitude;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.temperature){
		data[j] = Temperature;
		j++;
		datapackcount += 1;
		datalength += (4 + 1);
	}
	if(datatypewanted.distortion){
		data[j] = Distortion;
		j++;
		datapackcount += 1;
		datalength += (1 + 1);
	}
	if(datatypewanted.calstatus){
		data[j] = CalStatus;
		j++;
		datapackcount += 1;
		datalength += (1 + 1);
	}

	data[0] = set_data_components;//command to set data to recive
	data[1] = datapackcount;//count of data we expect
	
	compassSend(data, j);
	return(0);
}
