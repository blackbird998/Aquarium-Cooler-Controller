#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <OneWire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire ds(10); //pin 10

float temperatureSet = 25.0f; //zmienna przechowująca zadaną temperaturę
float tempC = 10.0f; //zmienna przechowująca zmierzoną temperaturę
unsigned long timeMain = 0; //główna zmienna wykorzystywana przy pomiarach czasu
bool interrupt = 1; //flaga sygnalizująca konieczność wykonania funkcji przerwania
int liquidLevel = 0; //funkcja przechowująca zmierzony poziom wody (0-1024)

void lcdInit(){
  Wire.setClock(10000);
  lcd.begin(16,2);   // Inicjalizacja LCD 2x16  
  lcd.backlight(); // zalaczenie podwietlenia 
  lcd.clear();
  lcd.setCursor(0,0); // Ustawienie kursora w pozycji 0,0 (pierwszy wiersz, pierwsza kolumna)
  lcd.print("LCD OK...");
  delay(2000);
  lcd.noBacklight();
}

float tempCheck() {//returns temperature in celcious (float)
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  
  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  //Serial.print("ROM =");
  //for( i = 0; i < 8; i++) {
    //Serial.write(' ');
    //Serial.print(addr[i], HEX);
  //}
 
  if (OneWire::crc8(addr, 7) != addr[7]) {
      //Serial.println("CRC is not valid!");
      return;
  }
  //Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28://
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  } 
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
 
  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();
 
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");

  //lcd.clear();
  

  return celsius;
}

void tempCheckManual(){
  
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp:");
  lcd.setCursor(7,0);
  lcd.print(tempC);
  
  lcd.setCursor(0,1);
  lcd.print("Set temp:");
  lcd.setCursor(10,1);
  lcd.print(temperatureSet);
  
  unsigned long time1 = millis();
  while(true){
    if(!digitalRead(8)) {//dekrementacja zadanej temperatury
      temperatureSet-=0.2f;
      lcd.setCursor(10,1);
      lcd.print(temperatureSet);
      delay(250);
      time1 = millis();
    }
    if(!digitalRead(9)) {//inkrementacja zadanej temperatury
      temperatureSet+=0.2f;
      lcd.setCursor(10,1);
      lcd.print(temperatureSet);
      delay(250);
      time1 = millis();
    }
    if(time1+5000<millis()) break;
  }
  lcd.noBacklight();
}

//==============================================
//https://fdossena.com/?p=ArduinoFanControl/i.md
//==============================================
//configure Timer 2 (pin 3) to output 25kHz PWM. Pin 11 will be unavailable for output in this mode
void setupTimer2(){
    //Set PWM frequency to about 25khz on pin 3 (timer 2 mode 5, prescale 8, count to 79)
    TIMSK2 = 0;
    TIFR2 = 0;
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << WGM22) | (1 << CS21);
    OCR2A = 79;
    OCR2B = 0;
}
//equivalent of analogWrite on pin 3
void setPWM2(float f){
    f=f<0?0:f>1?1:f;
    OCR2B = (uint8_t)(79*f);
}

void pwmSet(){
  liquidLevel=analogRead(14);
  delay(200);
  if(liquidLevel<550) setPWM2(0.0f);
  else if(tempC>temperatureSet) setPWM2(max(min((tempC-temperatureSet)*2.0f,0.8f),0.2f));//pwm duty min 0.2, max 0.8
  else if(tempC>temperatureSet-0.5f) setPWM2(0.2f);
  else setPWM2(0.0f);
}

void buttonInterrupt(){
  interrupt = 1;
}

void setup() {
  //===PWM===
  //enable outputs for Timer 2
  pinMode(3,OUTPUT); //Black5
  setupTimer2();
  setPWM2(0.0f); //set duty to 00% on pin 3
  //Buttons
  pinMode(8,INPUT); //Black14
  pinMode(9,INPUT); //Black15
  //Liquid Sensor
  pinMode(14,INPUT);
  
  lcdInit();
  attachInterrupt(digitalPinToInterrupt(2), buttonInterrupt, LOW); //Przerwanie na pinie 2
}

void loop() {
  if(interrupt == 1){
    do{
      tempC = tempCheck();
    }while(tempC<10.0f);
    tempCheckManual();
    pwmSet();
    interrupt = 0;
  }

  if(timeMain == 0) timeMain = millis();
  else if(timeMain + /*60000*/20000 < millis()){
    do{
      tempC = tempCheck();
    }while(tempC<10.0f);
    pwmSet();
    timeMain = 0;
      /*lcd.backlight();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Temp:");
      lcd.setCursor(7,0);
      lcd.print(tempC);
      
      lcd.setCursor(0,1);
      lcd.print("Set temp:");
      lcd.setCursor(10,1);
      lcd.print(temperatureSet);*/
  }

}
