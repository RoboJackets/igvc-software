#define RED_CTRL 2
#define GREEN_CTRL 3
#define BLUE_CTRL 4

#define B1S1_CTRL 5
#define B1S2_CTRL 6
#define B1S3_CTRL 7

#define B2S1_CTRL 8
#define B2S2_CTRL 9
#define B2S3_CTRL 10

#define ESTOP_IN 11

#define SAFETY_CTRL 12

#define BAT_IN A3

#define ERR_BAD_CTRL_PCKT 1 // Error code for malformed control packets

#define uchar unsigned char

#include <Servo.h>

/* Communication Protocol
 * The Arduino will loop, maintaining output states, until a new control
 * packet arrives over the serial connection. Recieving a control packet
 * causes the Arduino to change output states to match those requested
 * in the control packet, then return a response packet giving the
 * current input states.
 */

/* Control Packet Format:
 * Length : 12 Bytes
 * 0  - STX                                    (ASCII decimal 2)
 * 1  - Safety Light status                    (0 = solid, 1+ = blinking)
 * 2  - Underglow Color      [Red channel]     (range 0 - 255)
 * 3  - Underglow Color      [Green channel]   (range 0 - 255)
 * 4  - Underglow Color      [Blue channel]    (range 0 - 255)
 * 5  - Bar 1 Control        [Section 1]       (range 0 - 255)
 * 6  - Bar 1 Control        [Section 2]       (range 0 - 255)
 * 7  - Bar 1 Control        [Section 3]       (range 0 - 255)
 * 8  - Bar 2 Control        [Section 1]       (range 0 - 255)
 * 9  - Bar 2 Control        [Section 2]       (range 0 - 255)
 * 10 - Bar 2 Control        [Section 3]       (range 0 - 255)
 * 11 - EOT                                    (ASCII decimal 4)
 */
 
/* Response Packet Format:
 * Length : 4 Bytes
 * 0  - STX                                    (ASCII decimal 2)
 * 1  - EStop Status                           (0 = robot disabled, 1 = robot enabled)
 * 2  - Battery Level                          (range 0 - 255)
 * 3  - EOT                                    (ASCII decimal 4)
 */
 
/* Error Response Packet Format:
 * Length : 4 Bytes
 * 0 - STX                                     (ASCII decimal 2)
 * 1 - NAK                                     (ASCII decimal 21)
 * 2 - Error code                              (range 0 - 255)
 * 3 - EOT                                     (ASCII decimal 4)
 */

void parseControlPacket( uchar pckt[] )
{
  digitalWrite(SAFETY_CTRL, ( pckt[1] ? LOW : HIGH ) );
  analogWrite(RED_CTRL,   255 - pckt[2]);
  analogWrite(GREEN_CTRL, 255 - pckt[3]);
  analogWrite(BLUE_CTRL,  255 - pckt[4]);
  analogWrite(B1S1_CTRL,  pckt[5]);
  analogWrite(B1S2_CTRL,  pckt[6]);
  analogWrite(B1S3_CTRL,  pckt[7]);
  analogWrite(B2S1_CTRL,  pckt[8]);
  analogWrite(B2S2_CTRL,  pckt[9]);
  analogWrite(B2S3_CTRL,  pckt[10]);
}

void sendResponsePacket()
{
  uchar pckt[] = { 2, 0, 0, 4 };
  pckt[1] = digitalRead(ESTOP_IN);
  pckt[2] = analogRead(BAT_IN) / 4;
  Serial.write(pckt, 4);
}

void sendErrorPacket( int errorCode )
{
  uchar pckt[] = { 2, 21, 0, 4 };
  pckt[2] = errorCode;
  Serial.write(pckt, 4);
}

void setup()
{
  pinMode(RED_CTRL,    OUTPUT);
  pinMode(GREEN_CTRL,  OUTPUT);
  pinMode(BLUE_CTRL,   OUTPUT);
  pinMode(B1S1_CTRL,   OUTPUT);
  pinMode(B1S2_CTRL,   OUTPUT);
  pinMode(B1S3_CTRL,   OUTPUT);
  pinMode(B2S1_CTRL,   OUTPUT);
  pinMode(B2S2_CTRL,   OUTPUT);
  pinMode(B2S3_CTRL,   OUTPUT);
  pinMode(SAFETY_CTRL, OUTPUT);
  pinMode(ESTOP_IN,    INPUT );
  pinMode(BAT_IN,      INPUT );
  Serial.begin(9600);
  digitalWrite(SAFETY_CTRL, HIGH);
  analogWrite(RED_CTRL, 255);
  analogWrite(GREEN_CTRL, 255);
  analogWrite(BLUE_CTRL, 255);
  analogWrite(B1S1_CTRL, LOW);
  analogWrite(B1S2_CTRL, LOW);
  analogWrite(B1S3_CTRL, LOW);
  analogWrite(B2S1_CTRL, LOW);
  analogWrite(B2S2_CTRL, LOW);
  analogWrite(B2S3_CTRL, LOW);
}

/*
 * loop() waits for STX to cross serial connection, then gathers
 * the rest of the 12-byte packet. If the EOT char is not found
 * in the last position of the packet, the packet is not correct,
 * and the appropriate error is sent back. If the packet is
 * formed correctly, loop() passes the packet to
 * parseControlPacket().
 */
void loop()
{
  if(!Serial.available())
  {
    return;
  }
  uchar first = Serial.read();
  if(first != 2)
  {
    return;
  }
  uchar pckt[12];
  pckt[0] = first;
  for(int i = 1; i < 12; i++)
  {
    while(!Serial.available()) {}
    pckt[i] = Serial.read();
  }
  if(pckt[11] != 4)
  {
    sendErrorPacket(ERR_BAD_CTRL_PCKT);
  }
  else
  {
    parseControlPacket(pckt);
    sendResponsePacket();
  }
}
