#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>
#include <avr/sleep.h>
//#include <avr/wdt.h>

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
  ADC замер напряжения питания - A0
  Нога для подключения питания делителя напряжения - 6
  //часы DS1307 - пины SDA, CLK
  LED - Nano internal (pin 13)
  SerialEnable PIN - 4

  Потребление ~10mA в режиме ожидания. Из-за периферии модуля ардуино.
*/

/*
  - Чип уходит в спящий режим между нажатиями на кнопки.
  - Для открытия нажать кнопку BTN_OPEN1_PIN
  - Для закрытия нажать кнопку BTN_CLOSE1_PIN 
  - Открытие\закрытие по порогу температур CELSIUM_LEVEL_OPEN / CELSIUM_LEVEL_CLOSE
  - Пин для ключа питания сервоприводов
  - Делитель напряжения запитан от пина. Для отключения и экономии энегрии.
  - Выход из спящего режима по нажатию кнопки и каждые 32 секунды по таймеру
*/

volatile bool flag_runMainLoop = true;
volatile bool flag_runOnTimerLoop = true;

#define SERIAL_ENABLE_PIN (4)
bool flag_GoSleep = true;

Servo SRV1;
#define SERVO1_PIN (9)
#define SERVO1_POWER_PIN (5)
#define SERVO_POWER_STATE_ENABLED HIGH
#define SERVO_POWER_STATE_DISABLED LOW
#define SERVO1_CLOSED_VAL (0)
#define SERVO1_OPENED_VAL (180)
#define SERVO1_DRIVE_TIME (4000)
#define SERVO_NONEEDMOVE_VAL (-1)

#define BTN_OPEN1_PIN (2)
#define BTN_CLOSE1_PIN (3)
#define BTN_STATE_PRESSED LOW
#define BTN_STATE_UNPRESSED HIGH

#define LED_PIN 13

#define ANALOG_PIN A0
#define PIN_VOLTAGE (6)
#define BATTERY_LOW_VOLTAGE 3.4
#define BATTERY_FULL_VOLTAGE 4.1
int an1;
float koef_multiply = 5.623;
float koef_divide = 1000;
float voltage;


#define ONE_WIRE1_PIN (10)
OneWire OneWirePort(ONE_WIRE1_PIN);
#define TEMP_9_BIT (9)
DallasTemperature DT(&OneWirePort);
DeviceAddress addr_DS18b20;
float lastCelsium;
boolean flag_TemperatureSensorError = true;

#define CELSIUM_LEVEL_OPEN  (27) 
#define CELSIUM_LEVEL_CLOSE (22)
//1 cycle = 0.01630 2sec
// 1000 ~ 15 сек
#define postscalerVal (2000)
volatile unsigned int postscale = postscalerVal;

ISR (TIMER2_OVF_vect){
  postscale--;
  if (postscale==0){
    flag_runMainLoop = true;
    flag_runOnTimerLoop = true;
  }
}

void onExtInterrupt(){
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

  init_ADC_BAT_MON();
  //disable_ADC_INT_REF();
  // servo PWM
  pinMode(SERVO1_POWER_PIN,OUTPUT);
  digitalWrite(SERVO1_POWER_PIN, SERVO_POWER_STATE_DISABLED);
  //servo1_attach();
  //if (digitalRead(SERIAL_ENABLE_PIN)== LOW){
    Serial.begin(115200);
    Serial.println(F("Setup"));
    //flag_GoSleep = false;
  //}
  init_ServoInitMoves();
  // OneWire Initialization
  init_Thermometer();
  
  postscale = postscalerVal;

  attachInterrupt(0,onExtInterrupt, FALLING); // use interrupt 0 (pin 2) and run function
  attachInterrupt(1,onExtInterrupt, FALLING);
}

void loop(){
  if (flag_runMainLoop){
    Serial.println("MainLoop");
    postscale = postscalerVal;
    flag_runMainLoop = false;
    setLED(HIGH);
    if (flag_runOnTimerLoop){
      flag_runOnTimerLoop = false;
      timerLoop();
    }
    mainLoop();
    batteryHealthLoop();
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

void batteryHealthLoop()
{
  measureBatteryVoltage();
  //TODO: battery charge control  
}

void timerLoop()
{
  //refresh temperature
  Serial.println(F(">timerLoop begin"));
  flag_TemperatureSensorError = !DT.isConnected( addr_DS18b20 );
  if (!flag_TemperatureSensorError){
    DT.requestTemperatures();
    lastCelsium = DT.getTempC(addr_DS18b20);
  }else{
    Serial.println(F("Error:Thermometer not found"));
  }
  //move window 
  Serial.print(F("Temperature:"));
  Serial.println((int)lastCelsium);
  Serial.println("Limits: ");// + String(CELSIUM_LEVEL_CLOSE) + "/" + String(CELSIUM_LEVEL_OPEN));
  Serial.println("Actual servo state: "+(String)SRV1.read());
  signed int valBySensor = getNeededWindowStateBySensors();
  Serial.print("New servo position: ");
  Serial.println((signed int)valBySensor);
  if (valBySensor != SERVO_NONEEDMOVE_VAL){
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
  servo1_attach();
  SRV1.write(value);
  delay(SERVO1_DRIVE_TIME);
  SRV1.detach();
  servoPower(SERVO_POWER_STATE_DISABLED);  
}

signed int getNeededWindowStateBySensors(){
  if (flag_TemperatureSensorError){
    return SERVO_NONEEDMOVE_VAL;
  }
  
  if (lastCelsium >= CELSIUM_LEVEL_OPEN){
    return SERVO1_OPENED_VAL;  
  }
  
  if (lastCelsium <= CELSIUM_LEVEL_CLOSE){
    return  SERVO1_CLOSED_VAL;
  }
  return SERVO_NONEEDMOVE_VAL;
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

void servo1_attach()
{
  SRV1.attach(SERVO1_PIN);
}

void init_ServoInitMoves(){
  // Servo init move
  //moveServo1ToValue(SERVO1_OPENED_VAL);
  moveServo1ToValue(SERVO1_CLOSED_VAL);

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
 
    attachInterrupt(0,onExtInterrupt, FALLING); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets LOW
    attachInterrupt(1,onExtInterrupt, FALLING);                                   
 
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
 
    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    //detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
    //detachInterrupt(1);      // wakeUpNow code will not be executed
                             // during normal running time.
}

//==================Thermometer================================
void init_Thermometer()
{
  Serial.println("init_Thermometer");
  DT.begin();
  
  DT.setWaitForConversion(true);// don't return from 'requestTemperatures()'  before conversion completed. no need delay
  
  DT.getAddress(addr_DS18b20,0);
  
  DT.setResolution(addr_DS18b20,TEMP_9_BIT);
  
  
}
//=============================================================
void init_ADC_BAT_MON ()
{
  analogReference(INTERNAL);
  pinMode(PIN_VOLTAGE,OUTPUT);
}

void disable_ADC_INT_REF()
{
  analogReference(DEFAULT);
}

void measureBatteryVoltage()
{
  init_ADC_BAT_MON ();
  digitalWrite(PIN_VOLTAGE,HIGH);
  an1 = analogRead(ANALOG_PIN);
  voltage =  (an1*koef_multiply)/koef_divide;
  Serial.print("Battery voltage: ");
  Serial.println(voltage,2);
  digitalWrite(PIN_VOLTAGE,LOW);
  disable_ADC_INT_REF();
}
