//initialize and declare variables
const int led_pin = 13; //led attached to this pin
const int button_pin = 2; //connect in pull-up
 
boolean button_state = HIGH; //sence current state of switch
boolean button_last_state = HIGH;
boolean button_changed = false;

long last_debounce_time = 0;  // the last time the output pin was toggled
long debounce_delay = 200;    // the debounce time; increase if the output flickers
 
 
void setup() {
 
  //set the mode of the pins...
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);
 
}//close void setup
 
 
void loop() {
 
  //sample the state of the button - is it pressed or not?
  button_last_state = button_state;
  button_state = digitalRead(button_pin);
  // unsigned long currentTime = millis();
  if(button_state != button_last_state){
    button_changed = true;
  }
  else{
    button_changed = false;
  }

  if(button_state==LOW && button_changed){
    digitalWrite(led_pin, HIGH); //turn LED on

    // lastDebounceTime = millis(); //set the current time
  }
  else if(button_state==HIGH && button_changed){
    digitalWrite(led_pin, LOW); //turn LED on
   
    // lastDebounceTime = millis(); //set the current time
  }
  



 
}//close void loop
