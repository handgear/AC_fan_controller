const byte ledPin = 13;
const byte motorPin = 8;
const byte interruptPin = 2;
volatile byte state = LOW;
volatile byte divider = LOW; //120Hz to 60Hz

void setup() {
  pinMode(motorPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), motor_speed, FALLING);
}

void loop() {
  digitalWrite(motorPin, state);
}

void motor_speed() { //50%
  divider = !divider;
  if(divider == HIGH){
    state = !state;  
  }
}
