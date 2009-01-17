//Header
import processing.serial.*;


//Init Vars==========================//
int timePass = 0;
int newDeltaT = 0; 
int Hz = 14;
int sincr = 0;
int rincr = 0;
byte data;
int index;
int Val;
Serial sPort;
float ang = 0.0;
int oneTime = 5;
int bootWait = 0;

// Store the last 64 values received so we can graph them.
int[][] values = new int[16][17];
int[][] OValues = new int[16][17];
PFont TFont;

void setup(){
  size(1024,512);
  println("Available serial ports:");
  println(Serial.list());
  //sPort = new Serial(this, Serial.list()[0], 9600); 
  sPort = new Serial(this, "COM4", 9600);
  //sPort.buffer(3);
  TFont = createFont("Lucida Console",10);
  textFont(TFont,10);
  //Wait For Arduino to Reset
  bootWait = millis();
  while(bootWait>millis()-1000){}
  //Run Instructions to Arduino
  //sPort.write('%');sPort.write(14);
  //sPort.write('@'); sPort.write('1'); sPort.write('*');

  sPort.write('%'); sPort.write(14);
  sPort.write('#'); sPort.write(1);
  sPort.write('$'); sPort.write(4);sPort.write(1);
  sPort.write('$'); sPort.write(0);sPort.write(255);
  sPort.write('$'); sPort.write(1);sPort.write(255);
  sPort.write('$'); sPort.write(2);sPort.write(255);
  sPort.write('$'); sPort.write(3);sPort.write(255);
  sPort.write('$'); sPort.write(5);sPort.write(255);
  sPort.write('$'); sPort.write(6);sPort.write(255);
  sPort.write('$'); sPort.write(7);sPort.write(255);
  sPort.write('$'); sPort.write(8);sPort.write(255);
  sPort.write('$'); sPort.write(9);sPort.write(255);
  sPort.write('$'); sPort.write(10);sPort.write(255);
  sPort.write('^'); sPort.write('1');
  
  //sPort.write('='); sPort.write(4); sPort.write(0);
  //sPort.write('_'); sPort.write(8); sPort.write(1); sPort.write(0);
  //sPort.write('!');sPort.write('1');
  
  
}

void draw()
{


  background(12);
  noFill();
  //Draw 'Home' Center
  stroke(128);
  ellipse(512,512-10,25,25);
  //Draw Dividers
  stroke(32);
  for (float j = 0.0; j <= 10.0; j++){
    line(512,502,512-800.0*cos(j/10.0*PI),502-800.0*sin(j/10.0*PI));
  }
  //Draw Intervals
  stroke(16);
  for (int j = 0; j <= 40; j++){
    ellipse(512,502,j*40,j*40); 
  }
  stroke(32);
  for (int j = 0; j <= 4; j++){
    ellipse(512,502,j*400,j*400); 
  }
  //Draw Baseline
  stroke(96);
  line(0,500,1024,500);
  //Draw Data onto Polar Coord
  for (float i = 0.0; i < 10; i++){
    int in = int(i);
    //Old Values
    stroke(64,0,0);
    line(512-float(OValues[in][1])*cos((i)/10.0*PI),502-float(OValues[in][1])*sin((i)/10.0*PI),512-float(OValues[in+1][1])*cos((i+1.0)/10.0*PI),502-float(OValues[in+1][1])*sin((i+1.0)/10.0*PI));
    //New Values
    for (int j = 1; j <=9; j++)
    {
      int TVal = (values[in][j]/j);
      int NVal = (values[in+1][j]/j);
      if (TVal >= 1000){      //Ignore Bad Values
        TVal = 0; 
      }
      if (NVal >= 1000){
        NVal = 0;
      }
      if (TVal == 0 && in > 0){
        TVal = ((values[in-1][j]/j)+(values[in+1][j]/j))/2;
        if (TVal == 0 && NVal != 0){
          print('.');
          stroke(0,128,128);                                                                //Error Line Color
          line(512,502,512-800.0*cos(float(in)/10.0*PI),502-800.0*sin(float(in)/10.0*PI));  //Error Line
        }
      }
      if (NVal == 0 && in < 10){
        NVal = ((values[in][j]/j)+(values[in+2][j]/j))/2; 
      }

      stroke(0,j*32,255-j*32);
      if (j == 1){
        stroke(255,0,0);
      }
      line(512-float(TVal)*cos((i)/10.0*PI),502-float(TVal)*sin((i)/10.0*PI),512-float(NVal)*cos((i+1.0)/10.0*PI),502-float(NVal)*sin((i+1.0)/10.0*PI));
    }
    //Value Difference
    stroke(128,128,128);
    line(512-float(values[in][1])*cos(i/10.0*PI),502-float(values[in][1])*sin(i/10.0*PI),512-float(OValues[in][1])*cos((i)/10.0*PI),502-float(OValues[in][1])*sin((i)/10.0*PI));    
  } 
  for (int i = 0; i <= 10; i++){
    //Draw Text
    text(str(i),512-420.0*cos((i)/10.0*PI),502-420.0*sin((i)/10.0*PI)); 
  }
  //TextBox
  stroke(255,255,255);
  rect(40,30,60,190);
  for (int i = 0; i <=10; i++){
    text(nf(i,2)+":"+nf(values[i][1],4),50,50+i*16); 
  }

  while (sPort.available() >= 3){
    //Get Serial Data
    serialEvent();
  }
}

void serialEvent()
{
  print("[p"+rincr+"] ");
  print("packet recieved-");
  int inByte0 = sPort.read();
  int inByte1 = sPort.read();
  int inByte2 = sPort.read();
  int index = inByte0;
  int val = inByte1<<8;
  val = val+inByte2;
  print(" RAW("+inByte0+"."+inByte1+"."+inByte2+")");
  println(" PROC{  "+index+"."+val+"  }");

  rincr++;
  OValues[index][1] = values[index][1];
  if (val != 0){
    values[index][1] = val/10;
  }
}


