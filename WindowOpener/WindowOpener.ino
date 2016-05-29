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
*/

/*
  - Чип уходит в спящий режим между нажатиями на кнопки.
  - Для открытия нажать кнопку BTN_OPEN1_PIN
  - Для закрытия нажать кнопку BTN_CLOSE1_PIN 

  - Выход из спящего режима по нажатию кнопки и каждые 32 секунды по Watchdog
*/

volatile int wdt_counter = 0;
volatile bool flag_runMainLoop = true;
volatile bool flag_runOnTimerLoop = true;
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

#define CELSIUM_LEVEL_OPEN  30 
#define CELSIUM_LEVEL_CLOSE 20

// 1000 = 15 сек
#define postscalerVal (2000)
volatile unsigned int postscale = postscalerVal;
ISR (TIMER2_OVF_vect)
{
  postscale--;
  if (postscale==0){
    flag_runMainLoop = true;
    flag_runOnTimerLoop = true;
    postscale = postscalerVal;
    timer2_stop();
  }
}

void wakeUpNow()
{
   flag_runMainLoop =true;
}

void setup() 
{
  //init pins
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);
  
  pinMode(BTN_OPEN1_PIN,INPUT);
  digitalWrite (BTN_OPEN1_PIN,HIGH);// enable internal resistor
  pinMode(BTN_CLOSE1_PIN,INPUT);
  digitalWrite (BTN_CLOSE1_PIN,HIGH);// enable internal resistor
  // servo PWM
  pinMode(SERVO1_POWER_PIN,OUTPUT);
  digitalWrite(SERVO1_POWER_PIN, SERVO_POWER_STATE_DISABLED);
  SRV1.attach(SERVO1_PIN);
  Serial.begin(57600);
  //init_ServoInitMoves();
}

void loop()
{
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
  sleepNow();
}

void mainLoop()
{
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

void open_window1()
{
  moveServo1ToValue(SERVO1_OPENED_VAL);
}

void close_window1()
{
  moveServo1ToValue(SERVO1_CLOSED_VAL);
}

void moveServo1ToValue(byte value)
{
  servoPower(SERVO_POWER_STATE_ENABLED);
  SRV1.write(value);
  delay(SERVO1_DRIVE_TIME);
  servoPower(SERVO_POWER_STATE_DISABLED);  
}

void timerLoop()
{
  //refresh temperature
  Serial.println(F(">timerLoop begin"));
  readDS18B20Scratchpad();
  lastCelsium = getTemperatureCelsium();
  //move window 
  Serial.print(F("Temperature:"));
  Serial.print((int)lastCelsium); 
  Serial.flush(); 
  //Serial.println("; Limits: ");// + String(CELSIUM_LEVEL_CLOSE) + "/" + String(CELSIUM_LEVEL_OPEN));
  moveServo1ToValue(getNeededWindowStateBySensors());
  
  Serial.println(">timerLoop End");
}

byte getNeededWindowStateBySensors()
{
  if (lastCelsium >= CELSIUM_LEVEL_OPEN){
    return SERVO1_OPENED_VAL;  
  }
  
  if (lastCelsium <= CELSIUM_LEVEL_CLOSE){
    return  SERVO1_CLOSED_VAL;
  }
}

void setup_Timer2()
{
  TIMSK2=0x00;
  TCCR2B=0x00;
  TCCR2A=0x00;
  TIFR2=0x00;

  TCNT2= 0x00;         //reset timer count to 125 out of 255. 
  TCCR2B=0x07;     //using a prescaler of 6 to use divisor 1024.
  TIMSK2=1;      //timer2 Interrupt Mask Register. Set TOIE(Timer Overflow Interrupt Enable).

  postscale = postscalerVal;
}

void timer2_stop()
{
  TCCR2B = 0;// no clock input
}

bool onButton_Open()
{
  return ((digitalRead(BTN_OPEN1_PIN) == BTN_STATE_PRESSED) && (digitalRead(BTN_CLOSE1_PIN) != BTN_STATE_PRESSED));
}

bool onButton_Close()
{
  return ((digitalRead(BTN_CLOSE1_PIN) == BTN_STATE_PRESSED) && (digitalRead(BTN_OPEN1_PIN) != BTN_STATE_PRESSED));
}

void servoPower(int value)
{
  digitalWrite(SERVO1_POWER_PIN,value);
}

void setLED(int state)
{
    digitalWrite(LED_PIN, state);
}

void init_ServoInitMoves()
{
  // Servo init move
  servoPower(SERVO_POWER_STATE_ENABLED);
  SRV1.write(SERVO1_OPENED_VAL);
  delay(SERVO1_DRIVE_TIME);
  SRV1.write(SERVO1_CLOSED_VAL);
  servoPower(SERVO_POWER_STATE_DISABLED);

}

void sleepNow()         // here we put the arduino to sleep
{
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
void setTemperatureResolution()
{
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
}

float getTemperatureCelsium()
{
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

