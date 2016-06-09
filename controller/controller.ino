#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <DHT.h>

#define TEM_THRESHOLD 1 //Temperature threshold
#define HUM_THRESHOLD 3 //Humidity threshold
//=========pin connections===========//
const uint8_t fan_pin[5] = {22, 24, 26, 28, 30};
const uint8_t sensor_pin[5] = {8, 9, 10, 11, 12};
const uint8_t mux_pin_y = 34;
const uint8_t mux_pin_s0 = 35;
const uint8_t mux_pin_s1 = 36;
const uint8_t mux_pin_s2 = 37;
const uint8_t page_button = 3; //fan display select button, interrupt pin
const uint8_t zero_cross_pin = 2; //zero crossing detection, interrupt pin

const uint8_t latchPin = 39; //ST_CP of 74HC595
const uint8_t clockPin = 40; //SH_CP of 74HC595
const uint8_t dataPin = 38; //DS of 74HC595

//=========global variables===========//
float humidity[5]; //uint8_t will be fine. auto cascading
float temperature[5];
float humidity_old[5]; //uint8_t will be fine. auto cascading
float temperature_old[5];
uint8_t humidity_set[5];
float temperature_set[5];

uint8_t LedData1 = 0;
uint8_t LedData2 = 0;
uint8_t button_state[6]; //fan 1,2,3,4,5 / fan page select
uint8_t button_state_old[6]; //fan 1,2,3,4,5 / fan page select

uint8_t sensor_changed = 0;
uint8_t lcd_page_changed = 0;
// uint8_t fan_state_changed = 0;

//use volatile for interrupt variables
volatile uint8_t zero_cross_state = LOW;
volatile uint8_t lcd_page = 1; //1~5
volatile uint8_t fan_state[5] = {0,0,0,0,0}; //can be 0(off),1,2,3(full)
volatile uint8_t fan_state_old[5] = {0,0,0,0,0};
volatile uint8_t state = LOW;
volatile uint8_t divider = LOW; //120Hz to 60Hz

//=========object define===========//
LiquidCrystal_I2C lcd(0x3F,16,2);
Encoder left_encoder(18, 41); //18 == interrupt pin
Encoder right_encoder(19, 42); //19 == interrupt pin
DHT sensor1(sensor_pin[0], DHT22);
DHT sensor2(sensor_pin[1], DHT22);
DHT sensor3(sensor_pin[2], DHT22);
DHT sensor4(sensor_pin[3], DHT22);
DHT sensor5(sensor_pin[4], DHT22);

void setup(){
	// pinMode(fanPin[0], OUTPUT); 
  	pinMode(zero_cross_pin, INPUT_PULLUP);
  	attachInterrupt(digitalPinToInterrupt(zero_cross_pin), fan_speed_control, FALLING);
  	pinMode(page_button, INPUT_PULLUP);
  	attachInterrupt(digitalPinToInterrupt(page_button), INT_page_button, FALLING);
	pinMode(mux_pin_y, INPUT);
	pinMode(mux_pin_s0, OUTPUT);
	pinMode(mux_pin_s1, OUTPUT);
	pinMode(mux_pin_s2, OUTPUT);
	pinMode(latchPin, OUTPUT);

	sensor1.begin();
	sensor2.begin();
	sensor3.begin();
	sensor4.begin();
	sensor5.begin();
	lcd.init();
 	lcd.backlight();
  	lcd.print("test");
 	update_LCD();
}

void loop(){
	// read_sensor();

	check_button();

	if(fan_state_changed()){
		update_led();
		update_LCD();
	}

	if(lcd_page_changed == 1){
		update_LCD();
		lcd_page_changed = 0;
	}
	

	// digitalWrite(latchPin, 0);
	// shiftOut(dataPin, clockPin, LedData2);
	// shiftOut(dataPin, clockPin, LedData1);
	// digitalWrite(latchPin, 1);

	
	// delay(100);
  
	
}

void update_LCD(){
	//FanX  25.8C  63%
	lcd.clear();
	lcd.print("Fan"); lcd.print(lcd_page, DEC);
	lcd.setCursor(6, 0); lcd.print(temperature_old[lcd_page-1]); 
	lcd.setCursor(10, 0); lcd.print("C");
	lcd.setCursor(13, 0); lcd.print(humidity_old[lcd_page-1]); 
	lcd.setCursor(15, 0); lcd.print("%");
	lcd.setCursor(6, 1); lcd.print(temperature_set[lcd_page-1]); 
	lcd.setCursor(10, 1); lcd.print("C");
	lcd.setCursor(13, 1); lcd.print(humidity_set[lcd_page-1]); 
	lcd.setCursor(15, 1); lcd.print("%");
	if(fan_state[lcd_page-1] == 4){ //AI mode
		lcd.setCursor(1,1); lcd.print("AI");
	}
}

void INT_page_button(){ //need debouncing
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	// If interrupts come faster than 200ms, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > 300){
	  	lcd_page = lcd_page + 1;
		if(lcd_page == 6)
			lcd_page = 1;
	}
	last_interrupt_time = interrupt_time;
	lcd_page_changed = 1;
}
void fan_speed_control(){ //interrupt by zero crossing 
	divider = !divider;
  	if(divider == HIGH){
    	state = !state;  
  	}
}
void check_button(){
	digitalWrite(mux_pin_s0, LOW);
	digitalWrite(mux_pin_s1, LOW);
	digitalWrite(mux_pin_s2, LOW);
	button_state[0] = digitalRead(mux_pin_y);
	digitalWrite(mux_pin_s0, HIGH);
	digitalWrite(mux_pin_s1, LOW);
	digitalWrite(mux_pin_s2, LOW);
	button_state[1] = digitalRead(mux_pin_y);
	digitalWrite(mux_pin_s0, LOW);
	digitalWrite(mux_pin_s1, HIGH);
	digitalWrite(mux_pin_s2, LOW);
	button_state[2] = digitalRead(mux_pin_y);
	digitalWrite(mux_pin_s0, HIGH);
	digitalWrite(mux_pin_s1, HIGH);
	digitalWrite(mux_pin_s2, LOW);
	button_state[3] = digitalRead(mux_pin_y);
	digitalWrite(mux_pin_s0, LOW);
	digitalWrite(mux_pin_s1, LOW);
	digitalWrite(mux_pin_s2, HIGH);
	button_state[4] = digitalRead(mux_pin_y);
	digitalWrite(mux_pin_s0, HIGH);
	digitalWrite(mux_pin_s1, LOW);
	digitalWrite(mux_pin_s2, HIGH);
	button_state[5] = digitalRead(mux_pin_y);

	for(int i=0; i<5; i++){
		if(button_state[i] == HIGH && button_state_old[i] == LOW){
			if(fan_state[i] == 4)//full speed
				fan_state[i] = 0;
			else 
				fan_state[i] = fan_state[i] + 1;
		}	
		button_state_old[i] = button_state[i];
	}

}
void read_sensor(){//use _old values for display 
	humidity[0] = sensor1.readHumidity();
	humidity[1] = sensor2.readHumidity();
	humidity[2] = sensor3.readHumidity();
	humidity[3] = sensor4.readHumidity();
	humidity[4] = sensor5.readHumidity();
	temperature[0] = sensor1.readTemperature();
	temperature[1] = sensor2.readTemperature();
	temperature[2] = sensor3.readTemperature();
	temperature[3] = sensor4.readTemperature();
	temperature[4] = sensor5.readTemperature();

	for(int i=0; i<5; i++){//need to check overflow error
		//the conditions would need to be fixed
		if(humidity[i] - humidity_old[i] > HUM_THRESHOLD || 
			humidity[i] - humidity_old[i] < HUM_THRESHOLD){
			humidity_old[i] = humidity[i];
			sensor_changed = 1;
		}
		if(temperature[i] - temperature_old[i] > TEM_THRESHOLD || 
			temperature[i] - temperature_old[i] < TEM_THRESHOLD){
			temperature_old[i] = temperature[i];
			sensor_changed = 1;
		}
	}
}
uint8_t fan_state_changed(){
	if( fan_state[0] != fan_state_old[0] || 
		fan_state[1] != fan_state_old[1] ||
		fan_state[2] != fan_state_old[2] ||
		fan_state[3] != fan_state_old[3] ||
		fan_state[4] != fan_state_old[4]){

		fan_state_old[0] = fan_state[0];
		fan_state_old[1] = fan_state[1];
		fan_state_old[2] = fan_state[2];
		fan_state_old[3] = fan_state[3];
		fan_state_old[4] = fan_state[4];
		return 1;
	}
	else
		return 0;
}
void update_led(){
  if(fan_state[0]==0)
    LedData1 = LedData1 & B11111000;
  if(fan_state[0]==1){
  	LedData1 = LedData1 & B11111000;
    LedData1 = LedData1 | B00000001;
  }
  if(fan_state[0]==2){
  	LedData1 = LedData1 & B11111000;
    LedData1 = LedData1 | B00000011;
  }
  if(fan_state[0]==3 || fan_state[0]==4)
    LedData1 = LedData1 | B00000111;
//==============//
  if(fan_state[1]==0)
    LedData1 = LedData1 & B11000111;
  if(fan_state[1]==1){
  	LedData1 = LedData1 & B11000111;
    LedData1 = LedData1 | B00001000;
  }
  if(fan_state[1]==2){
	LedData1 = LedData1 & B11000111;
	LedData1 = LedData1 | B00011000;
  }
  if(fan_state[1]==3 || fan_state[1]==4)
    LedData1 = LedData1 | B00111000;
//=============//
  if(fan_state[2]==0){
  	LedData1 = LedData1 & B00111111;
    LedData2 = LedData2 & B11111110;
  }
  if(fan_state[2]==1){
  	LedData1 = LedData1 & B00111111;
    LedData1 = LedData1 | B01000000;
    LedData2 = LedData2 & B11111110;
  }
  if(fan_state[2]==2){
  	LedData1 = LedData1 & B00111111;
    LedData1 = LedData1 | B11000000; 
    LedData2 = LedData2 & B11111110;
  }
  if(fan_state[2]==3 || fan_state[2]==4){
  	LedData1 = LedData1 & B00111111;
    LedData1 = LedData1 | B11000000; 
    LedData2 = LedData2 & B11111110;
    LedData2 = LedData2 | B00000001;
  }
 //=============//
  if(fan_state[3]==0)
    LedData2 = LedData2 & B11110001;
  if(fan_state[3]==1){
  	LedData2 = LedData2 & B11110001;
    LedData2 = LedData2 | B00000010;
  }
  if(fan_state[3]==2){
  	LedData2 = LedData2 & B11110001;
    LedData2 = LedData2 | B00000110;
  }
  if(fan_state[3]==3 || fan_state[3]==4)
    LedData2 = LedData2 | B00001110;
//============//
  if(fan_state[4]==0)
    LedData2 = LedData2 & B10001111;
  if(fan_state[4]==1){
  	LedData2 = LedData2 & B10001111;
    LedData2 = LedData2 | B00010000;
  }
  if(fan_state[4]==2){
  	LedData2 = LedData2 & B10001111;
    LedData2 = LedData2 | B00110000;
  }
  if(fan_state[4]==3 || fan_state[4]==4)
    LedData2 = LedData2 | B01110000;

	digitalWrite(latchPin, 0);
	shiftOut(dataPin, clockPin, LedData2);
	shiftOut(dataPin, clockPin, LedData1);
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
