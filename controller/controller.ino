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

//use volatile for interrupt variables
volatile uint8_t zero_cross_state = LOW;
volatile uint8_t lcd_page = 1; //1~5
volatile uint8_t fan_state[5] = {0,0,0,0,0}; //can be 0(off),1,2,3(full)
volatile uint8_t fan_state_old[5] = {0,0,0,0,0};

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
  read_sensor();
	update_LCD();
  
  delay(500);
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

}

void INT_page_button(){ //need debouncing
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	// If interrupts come faster than 200ms, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > 500){
	  	lcd_page = lcd_page + 1;
		if(lcd_page == 6)
			lcd_page = 1;
	}
	last_interrupt_time = interrupt_time;
}
void fan_speed_control(){ //interrupt by zero crossing 

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
			if(fan_state[i] == 3)//full speed
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
		}
		if(temperature[i] - temperature_old[i] > TEM_THRESHOLD || 
			temperature[i] - temperature_old[i] < TEM_THRESHOLD){
			temperature_old[i] = temperature[i];
		}
	}
}
