//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <TimerOne.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <DHT.h>

#define TEM_THRESHOLD 10 //Temperature threshold
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
int8_t humidity[5]; 
int32_t temperature[5];
int8_t humidity_old[5];
int32_t temperature_old[5];
int8_t humidity_set[5] = {41,42,43,44,45};
int32_t temperature_set[5] = {10,260,270,280,290};

uint8_t LedData1 = 0;
uint8_t LedData2 = 0;
uint8_t button_state[6]; //fan 1,2,3,4,5 / fan page select
uint8_t button_state_old[6]; //fan 1,2,3,4,5 / fan page select

uint8_t sensor_changed = 0;
uint8_t lcd_data_changed = 0;
// uint8_t fan_state_changed = 0;
int32_t enc_temp1; //for debuging
int32_t enc_temp2;

//use volatile for interrupt variables
volatile uint8_t zero_cross_state = LOW;
volatile uint8_t lcd_page = 1; //1~5
volatile uint8_t fan_state[5] = {0,0,0,0,0}; //can be 0(off),1,2,3(full)
volatile uint8_t fan_state_old[5] = {0,0,0,0,0};
volatile uint8_t fan_speed_buf[5] = {0,0,0,0,0};
volatile uint8_t fan_state_auto[5] = {0,0,0,0,0};
volatile uint8_t state = LOW;
volatile uint8_t divider = LOW; //120Hz to 60Hz
volatile int32_t encoder_pos_L = 0;
volatile int32_t encoder_pos_R = 0;
volatile int32_t encoder_pos_old_L = 0;
volatile int32_t encoder_pos_old_R = 0;

//=========object define===========//
LiquidCrystal_I2C lcd(0x3F,16,2);
Encoder encoder_L(18, 41); //18 == interrupt pin //temperature encoder
Encoder encoder_R(19, 42); //19 == interrupt pin //humidity encoder
DHT sensor1(sensor_pin[0], DHT22);
DHT sensor2(sensor_pin[1], DHT22);
DHT sensor3(sensor_pin[2], DHT22);
DHT sensor4(sensor_pin[3], DHT22);
DHT sensor5(sensor_pin[4], DHT22);

void setup(){
	// pinMode(fan_pin[0], OUTPUT); 
  	pinMode(zero_cross_pin, INPUT_PULLUP);
  	attachInterrupt(digitalPinToInterrupt(zero_cross_pin), fan_speed_control, FALLING);
  	pinMode(page_button, INPUT_PULLUP);
  	attachInterrupt(digitalPinToInterrupt(page_button), INT_page_button, FALLING);
	pinMode(mux_pin_y, INPUT);
	pinMode(mux_pin_s0, OUTPUT);
	pinMode(mux_pin_s1, OUTPUT);
	pinMode(mux_pin_s2, OUTPUT);
	pinMode(latchPin, OUTPUT);
	Serial.begin(9600);
	Serial2.begin(9600); //bluetooth

	sensor1.begin();
	sensor2.begin();
	sensor3.begin();
	sensor4.begin();
	sensor5.begin();
	lcd.init();
 	lcd.backlight();
  	lcd.print("test");
 	update_LCD();
 	delay(1000);
 	Timer1.initialize(5000000); //set a timer for 5sec
  	Timer1.attachInterrupt(read_sensor); //call read_sensor()
}

void loop(){
	// read_sensor();

	check_button();
	// enc_temp1 = encoder_L.read(); //for debug
	// enc_temp2 = encoder_R.read();
	check_encoder();


	if(fan_state_changed()){
		update_led();
		update_LCD();
	}

	if(lcd_data_changed == 1){
		update_LCD();
		lcd_data_changed = 0;
	}
	


	
	// delay(100);
  
	
}

void update_LCD(){
	//FanX  25.8C  63%
	lcd.clear();
	lcd.print("Fan"); lcd.print(lcd_page, DEC);
	if(temperature_old[lcd_page-1]<100){
		lcd.setCursor(7, 0);
	}
	else
		lcd.setCursor(6, 0);
	lcd.print(temperature_old[lcd_page-1]); 
	lcd.setCursor(8, 0); lcd.print(".");
	lcd.setCursor(9, 0); lcd.print(temperature_old[lcd_page-1] % 10); 
	lcd.setCursor(10, 0); lcd.print("C");
	if(humidity_old[lcd_page-1]<10){
		lcd.setCursor(14, 0);
	}
	else
		lcd.setCursor(13, 0);
	lcd.print(humidity_old[lcd_page-1]); 
	lcd.setCursor(15, 0); lcd.print("%");

	if(temperature_set[lcd_page-1]<10){
		lcd.setCursor(7, 1); lcd.print("0");
		lcd.setCursor(8, 1);
	}
	else if(temperature_set[lcd_page-1]<100){
		lcd.setCursor(7, 1);
	}
	else
		lcd.setCursor(6, 1);
	lcd.print(temperature_set[lcd_page-1]);
	lcd.setCursor(8, 1); lcd.print(".");
	lcd.setCursor(9, 1); lcd.print(temperature_set[lcd_page-1] % 10); 
	lcd.setCursor(10, 1); lcd.print("C");

	if(humidity_set[lcd_page-1]<10){
		lcd.setCursor(14, 1);
	}
	else
		lcd.setCursor(13, 1);
	lcd.print(humidity_set[lcd_page-1]); 
	lcd.setCursor(15, 1); lcd.print("%");
	
	// lcd.setCursor(6, 1); lcd.print(enc_temp1); //for debug
	// lcd.setCursor(13, 1); lcd.print(enc_temp2);

	if(fan_state_auto[lcd_page-1] == 1){ //auto mode
		lcd.setCursor(0,1); lcd.print("Auto");
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
	lcd_data_changed = 1;
}
void fan_speed_control(){ //interrupt by zero crossing 
	divider = !divider;
  	if(divider == HIGH){ //every zero point of 60Hz
  		for(uint8_t i; i<5; i++){
  			if(fan_speed_buf[i]>0){
  				digitalWrite(fan_pin[i], HIGH); //turn_on();
				fan_speed_buf[i]--;
  			}
  			else{ //fan_speed_buf == 0
  				if(fan_state_old[i] == 1) //fan speed = low
  					fan_speed_buf[i] = 1; //50%
  				else if(fan_state_old[i] == 2) //fan speed = middle
  					fan_speed_buf[i] = 3; //75%
  				else if(fan_state_old[i] == 3) //fan speed = hi(full)
  					continue; //100%
  				digitalWrite(fan_pin[i], LOW); //turn_off();
  			}
  		}
 	}
}
void check_encoder(){
	encoder_pos_L = encoder_L.read(); //temperature encoder
	if(encoder_pos_old_L != encoder_pos_L){
		int32_t temp_buf = (encoder_pos_L - encoder_pos_old_L);
		temperature_set[lcd_page-1] += temp_buf;
		encoder_pos_old_L = encoder_pos_L;
		lcd_data_changed = 1;
	}

	encoder_pos_R = encoder_R.read(); //humidity encoder
	if(encoder_pos_old_R != encoder_pos_R){
		int8_t hum_buf = (encoder_pos_R - encoder_pos_old_R);
		humidity_set[lcd_page-1] += hum_buf;
		if(humidity_set[lcd_page-1]<0)
			humidity_set[lcd_page-1] = 0;
		encoder_pos_old_R = encoder_pos_R;
		lcd_data_changed = 1;
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
			if(fan_state[i] == 3 && fan_state_auto[i] == 0){ //enter auto mode
				fan_state_auto[i] = 1;
				fan_state[i] = fan_state_old[i];
				lcd_data_changed = 1;
			} 
			else if(fan_state_auto[i] == 1){ //exit auto mode
				fan_state_auto[i] = 0;
				fan_state[i] = 0;
				lcd_data_changed = 1;
			}
			else{
				fan_state[i] = fan_state[i] + 1;
			}
		}	
		button_state_old[i] = button_state[i];
	}

}
void read_sensor(){//use _old values for display 
	humidity[0] = (int)sensor1.readHumidity();
	humidity[1] = (int)sensor2.readHumidity();
	humidity[2] = (int)sensor3.readHumidity();
	humidity[3] = (int)sensor4.readHumidity();
	humidity[4] = (int)sensor5.readHumidity();
	temperature[0] = (int)(sensor1.readTemperature()*10);
	temperature[1] = (int)(sensor2.readTemperature()*10);
	temperature[2] = (int)(sensor3.readTemperature()*10);
	temperature[3] = (int)(sensor4.readTemperature()*10);
	temperature[4] = (int)(sensor5.readTemperature()*10);

	for(int i=0; i<5; i++){//need to check overflow error
		//the conditions would need to be fixed
		if(humidity[i] - humidity_old[i] > HUM_THRESHOLD || 
			humidity[i] - humidity_old[i] < HUM_THRESHOLD){
			humidity_old[i] = humidity[i];
			lcd_data_changed = 1;
		}
		if(temperature[i] - temperature_old[i] > TEM_THRESHOLD || 
			temperature[i] - temperature_old[i] < TEM_THRESHOLD){
			temperature_old[i] = temperature[i];
			lcd_data_changed = 1;
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
  if(fan_state[0]==3)
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
  if(fan_state[1]==3)
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
  if(fan_state[2]==3){
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
  if(fan_state[3]==3)
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
  if(fan_state[4]==3)
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
