/**
 * This is a sketch for an arduino board used to test serial communication.
 * When the program starts, it will open a serial connection at the baud 
 * rate 9600. As it loops, it will check for incoming data on the serial
 * connection. If it finds a capitol 'T' in the incoming stream, it will
 * toggle the state of pin 13. In most arduino boards, this pin is connected
 * to a built-in LED.
 * When ever the sketch does not find a 'T' in the incoming stream, it writes
 * the string "test read\n" to the serial connection every 1 second.
 * This sketch allows for testing both the tx and rx aspects of serial connections.
 * When serial communication is working properly, you should be able to turn the
 * pin 13 LED on/off by sending 'T' and should be able to read "test read\n" at 1
 * second intervals.
 * WARNING: The arduino board, like many devices, uses a special buffer to store
 * the serial stream until the appropriate receiver reads from it. It has been seen
 * that the arduino may crash if this buffer remains full for a long period of time.
 * The exact causes are unknown, but take this advice:
 * Do not send unnecessary amounts of data across serial (eg. reduce commands to one
 * character rather than full words) and be sure that both sides are reading from
 * the serial connection periodically.
 */

boolean state = false;
long lastMillis;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  lastMillis = millis();
}

void loop()
{
  if(Serial.available() > 0)
  {
    int inByte = Serial.read();
    if(inByte == 'T')
    {
      state = !state;
      if(state)
      {
        digitalWrite(13,HIGH);
      }
      else
      {
        digitalWrite(13, LOW);
      }
    }
  } else {
    if((millis() - lastMillis) > 1000)
    {
      Serial.println("test read");
      lastMillis = millis();
    }
  }
}

