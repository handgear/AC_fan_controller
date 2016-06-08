//Pin connected to ST_CP of 74HC595
int latchPin = 39;
//Pin connected to SH_CP of 74HC595
int clockPin = 40;
////Pin connected to DS of 74HC595
int dataPin = 38;

//led display data
byte LedData1 = 0;
byte LedData2 = 0;
byte fan_state[5] = {0,0,0,0,0}; //can be 0(off),1,2,3(full)
byte fan_state_old[5] = {0,0,0,0,0};

void setup() {
  //set pins to output because they are addressed in the main loop
  pinMode(latchPin, OUTPUT);
}

void loop() {

 

  // light each pin one by one using a function A
  for (int j = 0; j < 8; j++) {
    //ground latchPin and hold low for as long as you are transmitting
    digitalWrite(latchPin, 0);
    //red LEDs
    lightShiftPinA(7-j);
    //green LEDs
    lightShiftPinA(j);
    //return the latch pin high to signal chip that it 
    //no longer needs to listen for information
    digitalWrite(latchPin, 1);
    delay(1000);
  }



}

void update_led(void){
  if( fan_state[0] != fan_state_old[0] || )

  if(fan_state[0]==0)
    LedData1 = LedData1 & B00011111;
  if(fan_state[0]==1)
    LedData1 = LedData1 & B10011111 | B10000000;
  if(fan_state[0]==2)
    LedData1 = LedData1 & B11011111 | B11000000;
  if(fan_state[0]==3)
    LedData1 = LedData1 | B11100000;

  if(fan_state[1]==0)
    LedData1 = LedData1 & B11100011;
  if(fan_state[1]==1)
    LedData1 = LedData1 & B11110011 | B00010000;
  if(fan_state[1]==2)
    LedData1 = LedData1 & B11111011 | B00011000;
  if(fan_state[1]==3)
    LedData1 = LedData1 | B00011100;

  if(fan_state[2]==0)
    LedData1 = LedData1 & B11111100; LedData2 = LedData2 & B01111111;
  if(fan_state[2]==1)
    LedData1 = LedData1 & B11111110 | B00000010; LedData2 = LedData2 & B01111111;
  if(fan_state[2]==2)
    LedData1 = LedData1 | B00000011; LedData2 = LedData2 & B01111111;
  if(fan_state[2]==3)
    LedData1 = LedData1 | B00000011; LedData2 = LedData2 | B10000000;

  if(fan_state[3]==0)
    LedData2 = LedData2 & B10001111;
  if(fan_state[3]==1)
    LedData2 = LedData2 & B11001111 | B01000000;
  if(fan_state[3]==2)
    LedData2 = LedData2 & B11101111 | B01100000;
  if(fan_state[3]==3)
    LedData2 = LedData2 | B01110000;

  if(fan_state[4]==0)
    LedData2 = LedData2 & B11110001;
  if(fan_state[4]==1)
    LedData2 = LedData2 & B11111001 | B00001000;
  if(fan_state[4]==2)
    LedData2 = LedData2 & B11111101 | B00001100;
  if(fan_state[4]==3)
    LedData2 = LedData2 | B00001110;

 // 000/111/22 || 2/333/444 / (5)

  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin, LedData1);
  shiftOut(dataPin, clockPin, LedData2);
  digitalWrite(latchPin, 1);

}



void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  // This shifts 8 bits out MSB first, 
  //on the rising edge of the clock,
  //clock idles low

  //internal function setup
  int i=0;
  int pinState;
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, OUTPUT);

  //clear everything out just in case to
  //prepare shift register for bit shifting
  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);

  //for each bit in the byte myDataOut�
  //NOTICE THAT WE ARE COUNTING DOWN in our for loop
  //This means that %00000001 or "1" will go through such
  //that it will be pin Q0 that lights. 
  for (i=7; i>=0; i--)  {
    digitalWrite(myClockPin, 0);

    //if the value passed to myDataOut and a bitmask result 
    // true then... so if we are at i=6 and our value is
    // %11010100 it would the code compares it to %01000000 
    // and proceeds to set pinState to 1.
    if ( myDataOut & (1<<i) ) {
      pinState= 1;
    }
    else {  
      pinState= 0;
    }

    //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(myDataPin, pinState);
    //register shifts bits on upstroke of clock pin  
    digitalWrite(myClockPin, 1);
    //zero the data pin after shift to prevent bleed through
    digitalWrite(myDataPin, 0);
  }

  //stop shifting
  digitalWrite(myClockPin, 0);
}

