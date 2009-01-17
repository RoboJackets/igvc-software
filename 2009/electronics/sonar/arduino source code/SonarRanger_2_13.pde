//External Libraries
#include "Wire.h"          //Library for I2C Com
//===========Init Vars================
int dispSelect0 = 2;       //Pin ID of Display Selector 0
int dispSelect1 = 3;       //Pin ID of Display Selector 1
int dispSelect2 = 4;       //Pin ID of Display Selector 2
int SDA = PC5;             //Pin ID of SDA
int SCL = PC4;             //Pin ID of SCL
int dBit0 = 8;             //Pin ID of Display Bit 0
int dBit1 = 9;             //Pin ID of Display Bit 1
int dBit2 = 10;            //Pin ID of Display Bit 2
int dBit3 = 11;            //Pin ID of Display Bit 3    
int SonarProperties[16][7];//Sonar Properties Holder [addr][gain][maxDist][wThresh][seqId][active][rangeData]
int rangeMode = 0;         //Ranging Mode (0-individual,1-singleLoop,2-contLoop)
int readMode = 0;          //Reading Mode (0-individual,1-singleLoop,2-contLoop,3-contLoop_Warn)
int rangeId = -1;          //ID to Range During Range Cmd
int readId = -1;           //ID to Read During Read Cmd
int WARN_THRESH = 660;     //Default Threshold for warning mode
int Hz = 14;               //Default Seq's per Second
int TOTAL_STEPS = 10;      //Default total seq's
int ECHO_NUM = 1;          //Default Echo Num
int MAX_RANGE_DFLT = 4000; //Default Max Range Value
int GAIN_DFLT = 1;         //Default Gain Value
int ACTIVE_START = 0;      //First Active Sensor
int ACTIVE_END = 10;       //Last Active Sensor
                           //====================================


void setup(){                                          //=============Setup=================
  pinMode(dispSelect0,OUTPUT);                         //Set Display Selector 0 Pin as Out
  pinMode(dispSelect1,OUTPUT);                         //Set Display Selector 1 Pin as Out
  pinMode(dispSelect2,OUTPUT);                         //Set Display Selector 2 Pin as Out
  pinMode(dBit0,OUTPUT);                               //Set Display Bit 0 as Out
  pinMode(dBit1,OUTPUT);                               //Set Display Bit 1 as Out
  pinMode(dBit2,OUTPUT);                               //Set Display Bit 2 as Out
  pinMode(dBit3,OUTPUT);                               //Set Display Bit 3 as Out
  for (int i=0; i<=15; i++){                           //Setup SRF Addresses (0xE0/2 to 0xFE/2)
    SonarProperties[i][0] = 112+i;                     //ID 0 EQU 112 (0xE0/2), ID 1 EQU 113 (0xE2/2)       
  }                                                    //
  for (int i=0; i<=15; i++){                           //Setup DFLT warnthresh's
    SonarProperties[i][3] = WARN_THRESH;                //Set all Sonar Devices Warning Threshold to warnThresh
  }                                                    //
  for (int i=0; i<=15; i++){                           //Setup DFLT Seq IDs
    SonarProperties[i][4] = i;                         //Set all Sonar Devices to have sequential Seq IDs
  }                                                    //
  for (int i=0; i<=15; i++){                           //Activate Pre-Specified Sonar Devices
    if (i>=ACTIVE_START && i<=ACTIVE_END){             //For all Devices between Active_Start to Active_End
      SonarProperties[i][5] = 1;                       //Set Sonar Device as Active
    }else{                                             //Otherwise
      SonarProperties[i][5] = 0;                       //Set Sonar Device as Inactive
    }                                                  //
  }                                                    //
  Wire.begin();                                        //Initiate Wire Com
  Serial.begin(9600);                                  //Initiate Serial Com
  SetMaxRange(0x00,Range_MmToBin(MAX_RANGE_DFLT));       //Set Default Max Range for All Sonar Devices
  SetMaxGain(0x00,GAIN_DFLT);                           //Set Default Max Gain for All Sonar Devices
}                                                      //===================================


void loop(){                                           //=================MAIN======================
  ReadFromSerial();                                    //Read and Process from Serial Port  
  RangeCmd(rangeMode,rangeId);                         //Send Vals through RangeCmd
  ReadCmd(readMode,readId);                            //Send Vals through ReadCmd
  //UpdateDisplay(SonarProperties[5][6]);                //Show this Val on Breadboard Display
  //StartRange(112);
  //delay(1000);
  UpdateDisplay(1);
}                                                      //===========================================


void ReadFromSerial(){                                          //===========Read From Serial====================
  if (Serial.available() >= 1){                                 //If Serial RangeData Available
    byte inByte = Serial.read();                                //Read First Byte (CMD)
    if (inByte == 60){                                          //If 'Read All' Command recieved
      readMode = 1;                                             //Tell ReadCmd to perform Single Loop
    }                                                           //
    if (inByte == 62){                                          //If 'Start Ranging All' Command recieved
      rangeMode = 1;                                            //Tell RangeCmd to perform Single Loop
      rangeId = -1;                                             //Reset rangeId
    }                                                           //
    if (inByte >= 65 && inByte <= 90){                          //If 'Start Individual Range' Command recieved
      rangeMode = 0;                                            //Tell RangeCmd to perform Single Range
      rangeId = inByte-65;                                      //Extract Index from inByte
    }                                                           //
    if (inByte >= 97 && inByte <= 122){                         //If 'Read Individual' Command recieved
      readId = inByte-97;                                       //Extract Index from inByte
    }                                                           //
    if (inByte == 94){                                          //If 'Autopilot' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      if (inByte2 == 48){                                       //If 'Off' Command recieved
        rangeMode = 0;                                          //Tell RangeCmd to Stop Ranging
        readMode = 0;                                           //Tell ReadCmd to Stop Reading
      }                                                         //
      if (inByte2 == 49){                                       //If 'On' Command recieved
        rangeMode = 2;                                          //Tell RangeCmd to start Continuous Range Loops
        readMode = 2;                                           //Tell ReadCmd to start Continuous Read Loops
      }                                                         //
      inByte2 = 0;                                              //Clear inByte2
    }                                                           //
    if (inByte == 37){                                          //If 'Change Freq' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      if (inByte2 > 100){                                       //If inByte2 above 100
        inByte2 = 100;                                          //Set inByte2 EQU to 100
      }                                                         //
      Hz = inByte2;                                             //Set Freq EQU to inByte2
      inByte2 = 0;                                              //Clear inByte2
    }                                                           //
    if (inByte == 38){                                          //If 'Set All Max Gain' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      if (inByte2 > 31){                                        //If inByte2 above 31 (for more information, see SRF08 datasheet)
        inByte2 = 31;                                           //Set inByte2 EQU to 31
      }                                                         //
      SetMaxGain(0x00,inByte2);                                 //Set Max Gain of All Sonar Devices EQU to inByte2
      inByte2 = 0;                                              //Clear inByte2
    }                                                           //
    if (inByte == 124){                                         //If 'Set All Max Range' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      byte inByte3 = Serial.read();                             //Read Next Byte
      int range = ByteSmasher(inByte2, inByte3);                //Combine inByte2 and inByte3 into range
      range = Range_MmToBin(range);                             //Convert from mm to bin (see SRF08 datasheet for details)
      SetMaxRange(0x00,range);                                  //Set Max Range of All Sonar Devices EQU to range
      inByte2 = 0;                                              //Clear inByte2
      inByte3 = 0;                                              //Clear inByte3
    }                                                           //
    if (inByte == 33){                                          //If 'Warning Mode' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      if (inByte2 == 48){                                       //If 'Off' Command recieved
        rangeMode = 0;                                          //Tell RangeCmd to Stop Ranging
        readMode = 0;                                           //Tell ReadCmd to Stop Reading
      }                                                         //
      if (inByte2 == 49){                                       //If 'On' Command recieved
        rangeMode = 2;                                          //Tell RangeCmd to start Continuous Read Loops
        readMode = 3;                                           //Tell ReadCmd to start Continuous Conditional Read Loops
      }                                                         //
      inByte2 = 0;                                              //Clear inByte2
    }                                                           //
    if (inByte == 61){                                          //If 'Set All WarnThresh' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      byte inByte3 = Serial.read();                             //Read Next Byte
      int value = ByteSmasher(inByte2, inByte3);                //Combine inByte2 and inByte3 into value
      for (int i=0;i<=15;i++){                                  //For all Sonar Devices
        SonarProperties[i][3] = value;                          //Set WarnThresh Values for All Sonar Devices EQU to value
      }                                                         //
      inByte2 = 0;                                              //Clear inByte2
      inByte3 = 0;                                              //Clear inByte3
    }                                                           //
    if (inByte == 95){                                          //If 'Set Individual WarnThresh' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      byte inByte3 = Serial.read();                             //Read Next Byte
      byte inByte4 = Serial.read();                             //Read Next Byte
      int value = ByteSmasher(inByte3, inByte4);                //Combine inByte3 and inByte4 into value
      SonarProperties[inByte2][3] = value;                      //Set Individual Sonar Device WarnThresh EQU to Value
      inByte2 = 0;                                              //Clear inByte2
      inByte3 = 0;                                              //Clear inbyte3
      inByte4 = 0;                                              //Clear inByte4
    }                                                           //
    if (inByte == 43){                                          //If 'Set Individual Gain' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      byte inByte3 = Serial.read();                             //Read Next Byte
      if (inByte3 > 31){                                        //If inByte3 above 31
        inByte3 = 31;                                           //Set inByte3 EQU to 31
      }                                                         //
      SonarProperties[inByte2][1] = inByte3;                    //Set Individual Sonar Device Gain EQU to inByte3 (on Arduino register)
      SetMaxGain(SonarProperties[inByte2][0],inByte3);          //Set Individual Sonar Device Gain EQU to inByte3 (on SRF)
      inByte2 = 0;                                              //Clear inByte2
      inByte3 = 0;                                              //Clear inByte2
    }                                                           //
    if (inByte == 45){                                          //If 'Set Individual Max Range' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      byte inByte3 = Serial.read();                             //Read Next Byte
      byte inByte4 = Serial.read();                             //Read Next Byte
      int range = ByteSmasher(inByte3, inByte4);                //Combine inByte3 and inByte4 into range
      range = Range_MmToBin(range);                             //Convert from mm to bin (see SRF08 datasheet for details)
      SonarProperties[inByte2][2] = range;                      //Set Individual Sonar Device Max Range EQU to range (on Arduino register)
      SetMaxRange(SonarProperties[inByte2][0],range);           //Set Individual Sonar Device Max Range EQU to range (on SRF)
      inByte2 = 0;                                              //Clear inByte2
      inByte3 = 0;                                              //Clear inByte3
      inByte4 = 0;                                              //Clear inByte4
    }                                                           //
    if (inByte == 35){                                          //If 'Set Total Steps' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
       TOTAL_STEPS = inByte2;                                       //Set TotalSeq EQU to inByte2
      inByte2 = 0;                                              //Clear inByte2
    }                                                           //
    if (inByte == 36){                                          //If 'Set Individual Step ID' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      byte inByte3 = Serial.read();                             //Read Next Byte
      SonarProperties[inByte2][4] = inByte3;                    //Set Individual Step ID EQU to inByte3
      inByte2 = 0;                                              //Clear inbyte2
      inByte3 = 0;                                              //Clear inByte3
    }                                                           //
    if (inByte == 64){                                          //If 'Set Individual Device Active' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      byte inByte3 = Serial.read();                             //Read Next Byte
      if (inByte2 == 48){                                       //If 'Off' Command recieved
        if (inByte3 == 42){                                     //If 'Apply To All' Command recieved
          for (int i=0;i<=15;i++){                              //For All Sonar Devices
            SonarProperties[i][5] = 0;                          //Deactivate All Sonar Devices
          }                                                     //
        }else{                                                  //Otherwise
          SonarProperties[inByte3][5] = 0;                      //Deactivate Individual Sonar Device
        }                                                       //
      }                                                         //
      if (inByte2 == 49){                                       //If 'On' Command recieved
        if (inByte3 == 42){                                     //If 'Apply To All' Command recieved
          for (int i=0;i<=15;i++){                              //For ALl Sonar Devices
            SonarProperties[i][5] = 1;                          //Activate All Sonar Devices
          }                                                     //
        }else{                                                  //Otherwise
          SonarProperties[inByte3][5] = 1;                      //Activate Individual Sonar Device
        }                                                       //
      }                                                         //
      inByte2 = 0;                                              //Clear inByte2
      inByte3 = 0;                                              //Clear inbyte3
    }                                                           //
    if (inByte == 126){                                         //If 'Set Echo Num' Command recieved
      byte inByte2 = Serial.read();                             //Read Next Byte
      ECHO_NUM = inByte2;                                        //Set EchoNum EQU to inByte2
      inByte2 = 0;                                              //Clear inByte2
    }                                                           //
    inByte = 0;                                                 //Clear inByte
  }                                                             //
}                                                               //===============================================

void ReadCmd(int mode, int id){                                                                                         //=========ReadCmd============================
  if (mode == 0){                                                                                                       //If mode is EQU to 'Individual Read'
    if (id != -1){                                                                                                      //If id is NOT EQU to -1
      if (SonarProperties[id][5] == 1){                                                                                 //If Specified Sonar Device is Active
        SonarProperties[id][6] = GetRange(SonarProperties[id][0],ECHO_NUM);                                             //Get Range Value from Last Ping
        SendRangeToProcessing(id,SonarProperties[id][6]);                                                               //Send Range Value to Computer
      }                                                                                                                 //
      readMode = 0;                                                                                                     //Reset readMode 
      readId = -1;                                                                                                      //Reset readId
    }                                                                                                                   //
  }                                                                                                                     //
  if (mode == 1){                                                                                                       //If mode is EQU to 'Single Read Loop'
    for (int i=0;i<=15;i++){                                                                                            //For All Sonar Devices
      if (SonarProperties[i][5] == 1){                                                                                  //If Specified Sonar Device is Active
        SonarProperties[i][6] = GetRange(SonarProperties[i][0],ECHO_NUM);                                                //Get Range Value from Last Ping
        SendRangeToProcessing(i,SonarProperties[i][6]);                                                                 //Send Range Value to Computer
      }                                                                                                                 //
    }                                                                                                                   //
    readMode = 0;                                                                                                       //Reset readMode
    readId = -1;                                                                                                        //Reset readId
  }                                                                                                                     //
  if (mode == 2){                                                                                                       //If mode is EQU to 'Continuous Read Loop'
    for (int i=0;i<=15;i++){                                                                                            //For All Sonar Devices
      if (SonarProperties[i][5] == 1){                                                                                  //If Specified Sonar Device is Active
        SonarProperties[i][6] = GetRange(SonarProperties[i][0],ECHO_NUM);                                                //Get Range Value from Last Ping
        SendRangeToProcessing(i,SonarProperties[i][6]);                                                                 //Send Range Value to Computer
      }                                                                                                                 //
    }                                                                                                                   //
  }                                                                                                                     //
  if (mode == 3){                                                                                                       //If mode is EQU to 'Continuous Conditional Read Loop'
    for(int i=0;i<=15;i++){                                                                                             //For All Sonar Devices
      SonarProperties[i][6] = GetRange(SonarProperties[i][0],ECHO_NUM);                                                  //Get Range Value of Last Ping
      if (SonarProperties[i][6] <= SonarProperties[i][3] && SonarProperties[i][6] != 0 && SonarProperties[i][5] == 1){  //If Range Value is LT Individual WarnThresh and Range Value is NOT EQU to 0, and Specified Sonar Device is Active
        SendRangeToProcessing(i,SonarProperties[i][6]);                                                                 //Send Range Value to Computer
      }                                                                                                                 //
    }                                                                                                                   //
  }                                                                                                                     //
}                                                                                                                       //=========================================

void RangeCmd(int mode, int id){                                        //===========RangeCmd======================
  if (mode == 0){                                                       //If mode is EQU to 'Individual Range'
    if (id != -1){                                                      //If id is NOT EQU to -1
      if (SonarProperties[id][5] == 1){                                 //If Specified Sonar Device is Active
        StartRange(SonarProperties[id][0]);                             //Ping Specified Sonar Device
      }                                                                 //
      rangeMode = 0;                                                    //Reset rangeMode
      rangeId = -1;                                                     //Reset rangeId
    }                                                                   //
  }                                                                     //
  if (mode == 1){                                                       //If mode is EQU to 'Single Range Loop'
    for (int i=0; i<=TOTAL_STEPS; i++){                                    //For All Steps of Loop
      for (int j=0; j<=15; j++){                                        //For All Sonar Devices
        if (SonarProperties[j][4] == i && SonarProperties[j][5] == 1){  //If Step Id of Specified Sonar Device is THIS, and Specified Sonar Device is Active
          StartRange(SonarProperties[j][0]);                            //Ping Specified Sonar Device
        }                                                               //
      }                                                                 //
      delay(1000/Hz);                                                   //Pause for Specified Time
    }                                                                   //
    rangeMode = 0;                                                      //Reset rangeMode
    rangeId = -1;                                                       //Reset rangeId
  }                                                                     //
  if (mode == 2){                                                       //If mode is EQU to 'Continuous Range Loop'
    for (int i=0; i<=TOTAL_STEPS; i++){                                    //For All Steps of Loop
      for (int j=0; j<=15; j++){                                        //For All Sonar Devices
        if (SonarProperties[j][4] == i && SonarProperties[j][5] ==1){   //If Step Id of Specified Sonar Device is THIS, and Specified Sonar Device is Active
          StartRange(SonarProperties[j][0]);                            //Ping Specified Sonar Device
        }                                                               //
      }                                                                 //
      delay(1000/Hz);                                                   //Pause for Specified Time
    }                                                                   //
  }                                                                     //
}                                                                       //===========================================================

void SendRangeToProcessing(int index, int Val){  //========SendRangeToProcessing=============================
  byte bIndex = index;                           //Set bIndex EQU to index
  byte lbVal = Val;                              //Set lbVal EQU to Lower Byte of Val
  byte ubVal = (Val >> 8);                       //Set ubVal EQU to Upper Byte of Val
  Serial.print(bIndex);                          //Send Index of SRF08 to Computer
  Serial.print(ubVal);                           //Send Upper Byte Value to Computer
  Serial.print(lbVal);                           //Send Lower Byte Value to Computer
}                                                //=========================================================================================

int Range_MmToBin(int val){                      //========Range_MmToBin====================================================================
  val = floor((val-43)/43);                      //Put Value through Alg (as specified by SRF08 Datasheet)
  return(val);                                   //Return val
}                                                //=========================================================================================

int ByteSmasher(byte byteUpper, byte byteLower){ //===========ByteSmasher===================================================================
      int returnInt = byteUpper;                 //Set returnInt EQU to byteUpper
      returnInt = returnInt << 8;                //Shift Bits to Upper Byte
      returnInt = returnInt+byteLower;           //Add byteLower to returnInt
      return(returnInt);                         //Return returnInt
}                                                //=========================================================================================                                 

void SetMaxRange(byte Addr, byte RangeVal){      //===========SetMaxRange===================================================================
  Wire.beginTransmission(Addr);                  //Begin Transmission with Sonar Device @ Specified Address
  Wire.send(0x02);                               //Open MaxRange Register (0x02)
  Wire.send(RangeVal);                           //Write MaxRange Value
  Wire.endTransmission();                        //End Current Transmission
}                                                //=========================================================================================

void SetMaxGain(byte Addr, byte RangeVal){       //===========SetMaxGain====================================================================
  Wire.beginTransmission(Addr);                  //Begin Transmission with Sonar Device @ Specified Address
  Wire.send(0x01);                               //Open MaxGain Register (0x01)
  Wire.send(RangeVal);                           //Write MaxGain Value
  Wire.endTransmission();                        //End Current Transmission
}                                                //=========================================================================================

void StartRange(int Addr){                       //==========StartRange=====================================================================
  Wire.beginTransmission(Addr);                  //Begin Transmission with Sonar Device @ Specified Address
  Wire.send(0x00);                               //Open Command Register (0x00)
  Wire.send(0x51);                               //Write 0x51 to Command Register (Ping and Save Value in cm)
  Wire.endTransmission();                        //End Current Transmission
}                                                //==========================================================================================

int GetRange(int Addr, int EchoNum){             //========GetRange==========================================================================
  Wire.beginTransmission(Addr);                  //Begin Transmission with Sonar Device @ Specified Address
  Wire.send(EchoNum*2);                          //Tell SRF08 which Register to Read from
  Wire.endTransmission();                        //End Current Transmission
  Wire.requestFrom(Addr,2);                      //Request RangeData from Selected SRF08 Register
  while (Wire.available()){                      //If RangeData Sent
    byte inByte0 = Wire.receive();                    //Read Next Byte
    byte inByte1 = Wire.receive();                    //Read Next Byte
    int range = ByteSmasher(inByte0, inByte1);   //Combine inByte0 and inByte1 into range
    range = range*10;                            //Convert range from cm to mm
    return(range);                               //Return range
  }                                              //
}                                                //===========================================================================================

//===============================THE REST TO BE TRASHED=======================================================================================

void UpdateDisplay(int val){
  SendDigit(GetDigit(val,0,3),5,dispSelect0,dBit0,dBit1,dBit2,dBit3);
  SendDigit(GetDigit(val,1,3),5,dispSelect1,dBit0,dBit1,dBit2,dBit3);
  SendDigit(GetDigit(val,2,3),5,dispSelect2,dBit0,dBit1,dBit2,dBit3);
}

int GetDigit(int val,int index, int digTotal){
  int temp = 0;                             //Init Vars
  temp = (val/pow(10,(digTotal-index-1)));  //Extract Digit
  temp = temp%10;                           //Extract Digit
  return(temp);                             //Return Value
}

void SendDigit(int val,int MilliHold, int DispAddr, int Bit0Addr, int Bit1Addr, int Bit2Addr, int Bit3Addr){
  int Bit3;                     //Save Bit Vals
  int Bit2;                     //
  int Bit1;                     //
  int Bit0;                     //
  Bit3 = floor((val%16)/8);     //Save Bit Vals
  Bit2 = floor((val%8)/4);      //
  Bit1 = floor((val%4)/2);      //
  Bit0 = floor((val%2)/1);      //
  digitalWrite(DispAddr,1);     //Turn on Selected Display
  digitalWrite(Bit3Addr,Bit3);  //Send Bits
  digitalWrite(Bit2Addr,Bit2);  //
  digitalWrite(Bit1Addr,Bit1);  //
  digitalWrite(Bit0Addr,Bit0);  //
  delay(MilliHold);             //Hold Display Val
  digitalWrite(DispAddr,0);     //Clear Display
}

