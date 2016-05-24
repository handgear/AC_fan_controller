#include <EEPROM.h>

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int addr = 0;
byte value;
byte check;
byte data[4] = {20,30,40,50};
byte buf[4];

void setup() {
  Serial.begin(9600);

  check = EEPROM.read(0);
  if(check == 1){//load previous data(saved before turn off)
    for(int i=0;i<4;i++){
      data[i] = EEPROM.read(i+1);
    }
  }
  delay(100);
}

void loop() {
  Serial.println("=====programm start======");
  //print current data
  Serial.print("data[] = ");
  for(int i=0;i<4;i++){
    Serial.print(data[i], DEC);
    Serial.print("\t");
  }
  Serial.println();


  for(int i=0;i<4;i++){
    buf[i] = i+10;
  }  
 

  //print received data
  Serial.print("buf = ");
  for(int i=0;i<4;i++){
    Serial.print(buf[i], DEC);
    Serial.print("\t");
  }
  Serial.println();

  for(int i=0;i<4;i++){
      EEPROM.write(i+1, buf[i]);
  }
  EEPROM.write(0, 1);//set update flag
  

  for(int i=0;i<4;i++){
      data[i] = EEPROM.read(i+1);
  }

  //print changed, stored data
  Serial.println("=====stored data======");
  Serial.print("data[] = ");
  for(int i=0;i<4;i++){
    Serial.print(data[i], DEC);
    Serial.print("\t");
  }
  Serial.println();

  


  



  // if (addr == EEPROM.length()) {
  //   addr = 0;
  // }

  delay(5000);
}

// EEPROM.write(addr, val);
// value = EEPROM.read(address);
