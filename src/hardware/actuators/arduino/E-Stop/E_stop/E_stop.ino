
int estoppin = 5; // E-Stop Pin
int currentstate = 0;
void setup() {      
  pinMode(estoppin, INPUT);  
  Serial.begin(9600);
}
void loop() {
  int readstate = digitalRead(estoppin); // Read e-stop state
  if(currentstate != readstate) { // Compare curretn with past state
  currentstate = readstate;// Update current state.
  Serial.print(readstate);
  }
}
