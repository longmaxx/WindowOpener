#include <DS1307.h>
#include <OneWire.h>
#include <Servo.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/*
 !!!Для работы WDT надо перепрошивать стандартный загрузчик в ардуино.
 https://geektimes.ru/post/255800/
*/

/*
  Board: Arduino Nano v 3.0
  Кнопка Открыть1 - пин 2
  Кнопка Закрыть1 - пин 3
  Ключ питания привода - пин 5
  Сервопривод1 - пин 9
  Термометр ds18b20 - пин 12
  //часы DS1307 - пины SDA, CLK
  LED - Nano internal (pin 13)
  SerialEnable PIN - 4
*/

/*
  - Чип уходит в спящий режим между нажатиями на кнопки.
  - Для открытия нажать кнопку BTN_OPEN1_PIN
  - Для закрытия нажать кнопку BTN_CLOSE1_PIN 
  - Открытие\закрытие по порогу температур CELSIUM_LEVEL_OPEN / CELSIUM_LEVEL_CLOSE
  - Пин для ключа питания сервоприводов
  
  - Выход из спящего режима по нажатию кнопки и каждые 32 секунды по таймеру
*/

volatile int wdt_counter = 0;
volatile bool flag_runMainLoop = true;
volatile bool flag_runOnTimerLoop = true;

#define SERIAL_ENABLE_PIN 4
bool flag_GoSleep = true;

Servo SRV1;
#define SERVO1_PIN 9
#define SERVO1_POWER_PIN 5
#define SERVO_POWER_STATE_ENABLED HIGH
#define SERVO_POWER_STATE_DISABLED LOW
#define SERVO1_CLOSED_VAL 0
#define SERVO1_OPENED_VAL 180
#define SERVO1_DRIVE_TIME 2000

#define BTN_OPEN1_PIN 2
#define BTN_CLOSE1_PIN 3
#define BTN_STATE_PRESSED LOW
#define BTN_STATE_UNPRESSED HIGH

#define LED_PIN 13

#define ONE_WIRE1_PIN 12
OneWire ds(ONE_WIRE1_PIN);
byte scratchpad[12];
float lastCelsium;
bool flag_OneWire_CRC8_ERROR;

#define CELSIUM_LEVEL_OPEN  27 
#define CELSIUM_LEVEL_CLOSE 22

// 1000 = 15 сек
#define postscalerVal (2000)
volatile unsigned int postscale = postscalerVal;

ISR (TIMER2_OVF_vect){
  postscale--;
  if (postscale==0){
    flag_runMainLoop = true;
    flag_runOnTimerLoop = true;
    postscale = postscalerVal;
    timer2_stop();
  }
}

void wakeUpNow(){
   flag_runMainLoop =true;
}

void setup()
{
  //init pins
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);
  pinMode(SERIAL_ENABLE_PIN,INPUT);
  digitalWrite(SERIAL_ENABLE_PIN,HIGH);// enable internal pull-up resistor
  
  pinMode(BTN_OPEN1_PIN,INPUT);
  digitalWrite (BTN_OPEN1_PIN,HIGH);// enable internal pull-up resistor
  pinMode(BTN_CLOSE1_PIN,INPUT);
  digitalWrite (BTN_CLOSE1_PIN,HIGH);// enable internal pull-up resistor
  // servo PWM
  pinMode(SERVO1_POWER_PIN,OUTPUT);
  digitalWrite(SERVO1_POWER_PIN, SERVO_POWER_STATE_DISABLED);
  SRV1.attach(SERVO1_PIN);
  if (digitalRead(SERIAL_ENABLE_PIN)== LOW){
    Serial.begin(57600);
    //flag_GoSleep = false;
  }
  init_ServoInitMoves();
}

void loop(){
  if (flag_runMainLoop){
    flag_runMainLoop = false;
    setLED(HIGH);
    if (flag_runOnTimerLoop){
      flag_runOnTimerLoop = false;
      timerLoop();
    }
    mainLoop();
    setup_Timer2();
    setLED(LOW);
  }
  if (flag_GoSleep){
    delay(100);
    sleepNow();
  }  
}

void mainLoop(){
  if (onButton_Open())
    {
      Serial.println(">Button Open pressed");
      open_window1();
    }
    else if (onButton_Close())
    {
      Serial.println(">Button Close pressed");
      close_window1();
    }
}

void timerLoop(){
  //refresh temperature
  Serial.println(F(">timerLoop begin"));
  readDS18B20Scratchpad();
  lastCelsium = getTemperatureCelsium();
  //move window 
  Serial.print(F("Temperature:"));
  Serial.print((int)lastCelsium);
  Serial.print(" CRC_OK:");
  Serial.println(!flag_OneWire_CRC8_ERROR); 
  //Serial.println("; Limits: ");// + String(CELSIUM_LEVEL_CLOSE) + "/" + String(CELSIUM_LEVEL_OPEN));
  signed char valBySensor = getNeededWindowStateBySensors();
  Serial.print("New servo position: ");
  Serial.println(valBySensor);
  if (valBySensor != -1){
    moveServo1ToValue((unsigned char)valBySensor);
  }  
  
  Serial.println(">timerLoop End");
}

void open_window1(){
  moveServo1ToValue(SERVO1_OPENED_VAL);
}

void close_window1(){
  moveServo1ToValue(SERVO1_CLOSED_VAL);
}

void moveServo1ToValue(byte value){
  servoPower(SERVO_POWER_STATE_ENABLED);
  SRV1.write(value);
  delay(SERVO1_DRIVE_TIME);
  servoPower(SERVO_POWER_STATE_DISABLED);  
}

signed char getNeededWindowStateBySensors(){
  if (flag_OneWire_CRC8_ERROR){
    return -1;
  }
  if (lastCelsium >= CELSIUM_LEVEL_OPEN){
    return SERVO1_OPENED_VAL;  
  }
  
  if (lastCelsium <= CELSIUM_LEVEL_CLOSE){
    return  SERVO1_CLOSED_VAL;
  }
  return -1;
}

void setup_Timer2(){
  TIMSK2=0x00;
  TCCR2B=0x00;
  TCCR2A=0x00;
  TIFR2=0x00;

  TCNT2= 0x00;         //reset timer count to 125 out of 255. 
  TCCR2B=0x07;     //using a prescaler of 6 to use divisor 1024.
  TIMSK2=1;      //timer2 Interrupt Mask Register. Set TOIE(Timer Overflow Interrupt Enable).

  postscale = postscalerVal;
}

void timer2_stop(){
  TCCR2B = 0;// no clock input
}

bool onButton_Open(){
  return ((digitalRead(BTN_OPEN1_PIN) == BTN_STATE_PRESSED) && (digitalRead(BTN_CLOSE1_PIN) != BTN_STATE_PRESSED));
}

bool onButton_Close(){
  return ((digitalRead(BTN_CLOSE1_PIN) == BTN_STATE_PRESSED) && (digitalRead(BTN_OPEN1_PIN) != BTN_STATE_PRESSED));
}

void servoPower(int value){
  digitalWrite(SERVO1_POWER_PIN,value);
}

void setLED(int state){
    digitalWrite(LED_PIN, state);
}

void init_ServoInitMoves(){
  // Servo init move
  servoPower(SERVO_POWER_STATE_ENABLED);
  SRV1.write(SERVO1_OPENED_VAL);
  delay(SERVO1_DRIVE_TIME);
  SRV1.write(SERVO1_CLOSED_VAL);
  delay(SERVO1_DRIVE_TIME);
  servoPower(SERVO_POWER_STATE_DISABLED);

}

void sleepNow(){         // here we put the arduino to sleep

    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */  
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);   // sleep mode is set here
 
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
 
    /* Now it is time to enable an interrupt. We do it here so an
     * accidentally pushed interrupt button doesn't interrupt
     * our running program. if you want to be able to run
     * interrupt code besides the sleep function, place it in
     * setup() for example.
     *
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
     *
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */
 
    attachInterrupt(0,wakeUpNow, FALLING); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets LOW
    attachInterrupt(1,wakeUpNow, FALLING);                                   
 
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
 
    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
    detachInterrupt(1);      // wakeUpNow code will not be executed
                             // during normal running time.
}

//==================Thermometer================================
void setTemperatureResolution(){
    ds.reset();
    ds.write(0xCC); // skip ROM
    ds.write(0x4E);///write scratchpad
    ds.write(0x00);//TH
    ds.write(0x00);//TL
    ds.write(0b01011111);//prefs
}
void readDS18B20Scratchpad(){
  byte i;
  ds.reset();
  ds.write(0xCC);
  //ds.reset();
  ds.write(0x44); // start conversion
  delay(400);     // wait conversion
  // we might do a ds.depower() here, but the reset will take care of it.
   
  ds.reset();
  ds.write(0xCC);//skip rom
  //ds.reset();    
  ds.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    scratchpad[i] = ds.read();
  }
  flag_OneWire_CRC8_ERROR = !((Make_CRC8(scratchpad,8) == (unsigned char)scratchpad[8]) && (scratchpad[8] != 0));
}


float getTemperatureCelsium(){
  byte type_s = false;
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (scratchpad[1] << 8) | scratchpad[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (scratchpad[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - scratchpad[6];
    }
  } else {
    byte cfg = (scratchpad[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  return (float)raw / 16.0;
  
}
//==================End Thermometer============================

//================= CRC Calculation==========================================
const unsigned char CRC8Table[256] = {
0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

/*
This procedure calculates the cumulative Dallas Semiconductor 1–Wire CRC of all bytes passed to it.
*/
unsigned char  Do_CRC8(unsigned char CRC, unsigned char X){
  return CRC8Table[CRC ^ X];//xor
}

unsigned char Make_CRC8(unsigned char* arr, unsigned char len){
  unsigned char tmpCRC8 = 0x00;
  for (unsigned char i=0;i<len;i++){
    tmpCRC8 = Do_CRC8(tmpCRC8,arr[i]);
  }
  return tmpCRC8;
}

