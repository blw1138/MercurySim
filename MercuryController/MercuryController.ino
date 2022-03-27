// Mercury Control Panel Controller
// Brett Williams
// March 2017

#include <SerialCommand.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Servo globe_servo;

//Define Pins
const int globeServo = 0;
const int globeMotor = 3;
const int ploadPin = 13;
const int switchLatchPin = 11;
const int ledLatchPin = 9;
const int clockPin = 12;
const int dataPin = 10;

//Shift Register Buffer
byte leds[3];
byte switches[3];

//Servo Data
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x65);
#define SERVOMIN  202 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  700 // this is the 'maximum' pulse length count (out of 4096)
int servo_pwm[16];
bool move_globe;

typedef struct {
  bool ledState;
  long onTime;
  unsigned long startMillis;
} LED;

#define LED_COUNT 24
#define DEFAULT_LED_TIME 8000

LED led_list[LED_COUNT];

SerialCommand SCmd;

void setup()
{
  pinMode(ploadPin, OUTPUT);
  pinMode(switchLatchPin, OUTPUT);
  pinMode(ledLatchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);

  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin, HIGH);
  all_leds_off();

  servos.begin();
  servos.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  serial_setup();

  move_globe = false;
}

void loop()
{
//  byte switchBuffer[3];
//  readSwitches(switchBuffer);
  update_leds();
  advance_globe();
  SCmd.readSerial();

  delay(20);
}

/* ==== SERIAL METHODS ==== */

void serial_setup()
{
  Serial.begin(9600);
  SCmd.addDefaultHandler(unrecognized);
  //LED Commands
  SCmd.addCommand("led", process_led_command);
  SCmd.addCommand("led_red", led_red);
  SCmd.addCommand("led_all_on", cycle_all_leds);
  SCmd.addCommand("led_all_off", all_leds_off);
  SCmd.addCommand("led_cycle", cycle_all_leds);
  //Servo Commands
  SCmd.addCommand("servo_zero", zero_all_servos);
  SCmd.addCommand("servo_min", zero_all_servos);
  SCmd.addCommand("servo_center", center_all_servos);
  SCmd.addCommand("servo_max", max_all_servos);
  SCmd.addCommand("servo", process_servo_command);
  //Globe Commands
  SCmd.addCommand("globe", globe);
  Serial.println(F("Mercury Control Ready"));
}

void unrecognized()
{
  Serial.println(F("Unrecognized Command"));
}

void process_led_command()
{
  int ledPin;
  String ledStatusText;
  bool ledStatus;
  char *arg;

  arg = SCmd.next();
  if (arg != NULL)
  {
    ledPin = atoi(arg);  // Converts a char string to an integer
  }
  else {
    Serial.println("Error: No pin number");
    return;
  }

  arg = SCmd.next();
  if (arg != NULL)
  {
    ledStatusText = String(arg);
    ledStatus = ledStatusText.equals("on");
  }
  else {
    Serial.println("Error: No pin status");
    return;
  }

  Serial.print(F("Turning LED #"));
  Serial.print(ledPin);
  if (ledStatus){
    Serial.println(" on");
  }else{
    Serial.println(" off");
  }

  set_led_state(ledPin, ledStatus, DEFAULT_LED_TIME);
}

void process_servo_command()    
{
  int servo_num;
  int percent;
  int duration = 0;
  char *arg; 
  
  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    servo_num=atoi(arg);    // Converts a char string to an integer
  } 
  else {
    Serial.println(F("Error: No servo number"));
    return; 
  }
  
  if (servo_num == globeServo){
    Serial.println(F("Error: Cannot move globe"));
    return;
  }

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    percent=atol(arg);
  } 
  else {
    Serial.println(F("Error: No servo move percent"));
    return;
  }

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    duration=atol(arg);
  }

  Serial.print(F("Moving servo #"));
  Serial.print(servo_num);
  Serial.print(" to ");
  Serial.print(percent);
  Serial.println("%");

  if (duration > 0){
    move_servo_to_percent_timed(servo_num, percent, duration);
  }
  else{
    move_servo_to_percent(servo_num, percent);
  }
}

/* ==== SWITCH METHODS ==== */

void readSwitches(byte switchBuffer[])
{
  pinMode(dataPin, INPUT);

  //Collect Data
  digitalWrite(switchLatchPin, HIGH);
  digitalWrite(ploadPin, LOW);
  delayMicroseconds(20);
  digitalWrite(ploadPin, HIGH);
  digitalWrite(switchLatchPin, LOW);

  //Shift data in
  switchBuffer[0] = shiftIn(dataPin, clockPin, MSBFIRST);
  switchBuffer[1] = shiftIn(dataPin, clockPin, MSBFIRST);
  switchBuffer[2] = shiftIn(dataPin, clockPin, MSBFIRST);

  //Look for any changes
  for (int x = 0; x < 3; x++) {
    for (int i = 0; i < 8; i++)
    {
      if (bitRead(switchBuffer[x], i) != bitRead(switches[x], i)) {
        int pin_number = (x * 8) + i;
        //set_pin_status(pin_number, 1);
        Serial.print("Pin - ");
        Serial.print(pin_number);
        if (bitRead(switchBuffer[x], i) > 0)
          Serial.print("HIGH");
        else
          Serial.print("LOW");
        Serial.print("\r\n");
      }
    }
    switches[x] = switchBuffer[x];
  }
}

/* ==== LED METHODS ==== */

void update_leds() {

  for (int pin = 0; pin < LED_COUNT; pin++) {
    if (led_list[pin].ledState == true) {
      long timeDiff = millis() - led_list[pin].startMillis;
      
      if (timeDiff > led_list[pin].onTime) {
        led_list[pin].ledState = false;
        led_list[pin].startMillis = 0;
        set_pin_status(pin, false);
        Serial.print("turning off led #");
        Serial.println(pin);
      }
    }
  }
}

void set_led_state(byte pin, bool onState, long duration) {

  if (onState == led_list[pin].ledState) {
    return;
  } else if ((onState == true) && (duration > 0)) {
    Serial.print("Turning on LED #");
    Serial.print(pin);
    Serial.print(" for ");
    Serial.print(duration / 1000);
    Serial.println(" seconds");
    led_list[pin].startMillis = millis();
    led_list[pin].onTime = duration;
    Serial.println(duration);
    set_pin_status(pin, true);
  } else {
    Serial.print("Turning off LED #");
    Serial.println(pin);
    led_list[pin].startMillis = 0;
    set_pin_status(pin, false);
  }

  led_list[pin].ledState = onState;
}

void cycle_all_leds()
{
  for (int x = 0; x < 3; x++)
  {
    for (int i = 0; i < 8; i++)
    {
      Serial.print("LED #");
      int led_num = (x * 8) + i;
      Serial.println(led_num);
      bitSet(leds[x], i);
      updateShiftRegister();
      delay(2000);
      bitClear(leds[x], i);
      updateShiftRegister();
      delay(20);
    }
  }
  Serial.println("Ready");
}

void all_leds_off()
{
  leds[0] = 0;
  leds[1] = 0;
  leds[2] = 0;
  updateShiftRegister();
}

void led_red(){
  set_led_state(2, true, DEFAULT_LED_TIME);
  set_led_state(7, true, DEFAULT_LED_TIME);
  set_led_state(8, true, DEFAULT_LED_TIME);
  set_led_state(11, true, DEFAULT_LED_TIME);
  set_led_state(13, true, DEFAULT_LED_TIME);
  set_led_state(15, true, DEFAULT_LED_TIME);
  set_led_state(16, true, DEFAULT_LED_TIME);
  set_led_state(19, true, DEFAULT_LED_TIME);
  set_led_state(21, true, DEFAULT_LED_TIME);
  set_led_state(23, true, DEFAULT_LED_TIME);
}

void set_pin_status(int pinNumber, bool pinStatus) {

  byte idx = 0;

  while (pinNumber >= 8) {
    pinNumber -= 8;
    idx++;
  }

  if (pinStatus == true) {
    //Serial.println("LED ON");
    bitSet(leds[idx], pinNumber);
  } else {
    //Serial.println("LED OFF");
    bitClear(leds[idx], pinNumber);
  }

  updateShiftRegister();
}

void updateShiftRegister()
{
  pinMode(dataPin, OUTPUT);
  digitalWrite(ledLatchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, leds[2]);
  shiftOut(dataPin, clockPin, MSBFIRST, leds[1]);
  shiftOut(dataPin, clockPin, MSBFIRST, leds[0]);
  digitalWrite(ledLatchPin, HIGH);
}

/* ==== SERVO METHODS ==== */

void zero_all_servos() {
  Serial.println("Setting all servos to zero");
  for (int x = 1; x < 16; x++) {
    servos.setPWM(x, 0, SERVOMAX);
    servo_pwm[x] = SERVOMAX;
    delay(200);
  }
  Serial.println("Ready");
}

void center_all_servos() {
  Serial.println("Setting all servos to center");
  for (int x = 1; x < 16; x++) {
    move_servo_to_percent(x, 50);
    delay(200);
  }
  Serial.println("Ready");
}

void max_all_servos() {
  Serial.println("Setting all servos to maximum");
  for (int x = 1; x < 16; x++) {
    servos.setPWM(x, 0, SERVOMIN);
    servo_pwm[x] = SERVOMIN;
    delay(200);
  }
  Serial.println("Ready");
}

void move_servo_to_degree(byte servo_num, int degrees) {
  int pulselength = map(degrees, 0, 180, SERVOMAX, SERVOMIN);
  servos.setPWM(servo_num, 0, pulselength);
  servo_pwm[servo_num] = pulselength;
}

void move_servo_to_percent(byte servo_num, int percentage) {
  int pulselength = map(percentage, 0, 100, SERVOMAX, SERVOMIN);
  servos.setPWM(servo_num, 0, pulselength);
  servo_pwm[servo_num] = pulselength;
}

void move_servo_to_percent_timed(byte servo_num, int percentage, int duration){
  int new_pulselength = map(percentage, 0, 100, SERVOMAX, SERVOMIN);
  int old_pulselength = servo_pwm[servo_num];
  int duration_millis = (duration * 1000) / 40; //200

  int millis_interval = (new_pulselength - old_pulselength) / duration_millis;
  Serial.println(new_pulselength);
  Serial.println(old_pulselength);

   for (int i=0; i <= duration_millis; i++){
      int pulselength = old_pulselength + (i * millis_interval);
      servos.setPWM(servo_num, 0, pulselength);
      delay(40);
   }
  servos.setPWM(servo_num, 0, new_pulselength);
  servo_pwm[servo_num] = new_pulselength;
  Serial.println("Ready");
}

/* ==== SERVO METHODS ==== */

void globe(){

  if (move_globe == true){
    move_globe = false;
    analogWrite(globeMotor, 0);
    move_servo_to_percent(0, 50);
    Serial.println("Globe movement off");
  }else{
    move_globe = true;
    analogWrite(globeMotor, 50);
    Serial.println("Globe movement on");
  }

}

void advance_globe() {

  // Orbital period 88.5 minutes - 5310 seconds
  // 6283 time slices
  // Updates every 845 milliseconds

  if (move_globe == false){
    return;
  }

  const int orbital_period = 5310;
  const int inclination = 90;
  const float sine_increment = 0.001;
  const float two_pi = 6.283;
  const float time_delta = (orbital_period / (two_pi / sine_increment)) * 1000;

  static float x = 0;
  static unsigned long last_update = 0;
  unsigned long current_millis = millis();

  // Update if time delta has passed or never updated
  if ((current_millis - last_update) >= time_delta || last_update == 0) {

    float val = sin(x) * inclination + inclination;

    move_servo_to_degree(globeServo, int(val));
    //HIGH_servo.write(int(val));

    x += sine_increment;

    // Rollover angle
    if (x >= two_pi) {
      x = 0;
    }

    last_update = current_millis;
  }
}

