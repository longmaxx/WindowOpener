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
  Термометр ds18b20 - пин 10
  часы DS1307 - пины SDA, CLK
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

// watchdog interrupt
//ISR(WDT_vect) 
//{
//  wdt_counter++;
//}

void wakeUpNow()
{
  // action in interrupt on wake from sleep
  wdt_disable();  // disable watchdog
  wdt_counter = 0;
  flag_runMainLoop = true;
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
  init_ServoInitMoves();
//  wdt_reset();
//  wdt_enable(WDTO_8S);
}

void loop()
{
  //wdt_disable();
  // put your main code here, to run repeatedly:
  if (wdt_counter>4){
    flag_runMainLoop = true;
  }
  if (flag_runMainLoop){
    flag_runMainLoop = false;
    setLED(HIGH);
    if (servo1_needOpen())
    {
      servoPower(SERVO_POWER_STATE_ENABLED);
      SRV1.write(SERVO1_OPENED_VAL);
      delay(SERVO1_DRIVE_TIME);
      servoPower(SERVO_POWER_STATE_DISABLED);
    }
    else if (servo1_needClose())
    {
      servoPower(SERVO_POWER_STATE_ENABLED);
      SRV1.write(SERVO1_CLOSED_VAL);
      delay(SERVO1_DRIVE_TIME);
      servoPower(SERVO_POWER_STATE_DISABLED);
    }
    setLED(LOW);
//    wdt_DoEnable();
//    wdt_counter = 0;
  }
}

//void wdt_DoEnable()
//{
//  wdt_disable();
//  wdt_enable(WDTO_8S);
//  wdt_reset();
//  //wdt_counter = 0;
//}

bool servo1_needOpen()
{
  return ((digitalRead(BTN_OPEN1_PIN) == BTN_STATE_PRESSED) && (digitalRead(BTN_CLOSE1_PIN) != BTN_STATE_PRESSED));
}

bool servo1_needClose()
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
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
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

