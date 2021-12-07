#include "Motors_c.h"               // Packages
#include "LineSensors_c.h"
#include "Imux_c.h"
#include "EEPROM3pi_c.h"
#include "PID_c.h"
#include <EEPROM.h>

#define LINE_SENSOR_UPDATE  100     // Update timings

#define MOTOR_PWR           50      // Proportional gain #1

#define LINE_THRESHOLD      1000    // A threshold for detecting whether there is a line or not

// Push buttons connected to:
#define BUTTON_A_PIN  14
#define BUTTON_B_PIN  30
#define BUTTON_C_PIN  17

Motors_c MTR;                       // Create instances of classes
LineSensors_c LS;
Imux_c IMUX;
EEPROM3pi_c EPRM;
PID_c LFPID;
PID_c XPPID;
PID_c HPID;

#define STATE_OFFLINE   0           // States for the state machine
#define STATE_ONLINE    1

int state;                          // This will hold which state your robot is in
unsigned long posttime;             // Timer for background x-position mapping


// put your setup code here, to run once:
void setup() {

  // Standard initialisations for the Motors and line sensor
  MTR.initialise();
  LS.initialise();
  IMUX.initialise();

  // Use two push buttons to decide
  // whether to read or write when the
  // 3Pi+ is turned on.
  // Here, INPUT_PULLUP means that the
  // pin will read HIGH by default, and
  // a button press will make it read LOW
  pinMode( BUTTON_A_PIN, INPUT_PULLUP );
  pinMode( BUTTON_B_PIN, INPUT_PULLUP );
  pinMode( BUTTON_C_PIN, INPUT_PULLUP );

  // Serial initialisation
  Serial.begin(9600);
  delay(1000);
  Serial.println("***Setup Complete***");

  // Wait for a button press to decide
  // what to do.
  int a;
  int b;
  int c;
  a = HIGH;
  b = HIGH;
  c = HIGH;
  do {
    Serial.println("A = write, B = read, C = start experiment");
    a = digitalRead(BUTTON_A_PIN);
    b = digitalRead(BUTTON_B_PIN);
    c = digitalRead(BUTTON_C_PIN);
  } while ( a == HIGH && b == HIGH && c == HIGH);

  // Saves our volatile grid into EEPROM.
  if ( a == LOW ) {
    Serial.println("Writing new values to EEPROM");
    EPRM.writeGridToEEPROM();
    while (1) {
      Serial.println("Write done, please reset");
      delay(100000);
    }
  } else if ( b == LOW ) { // Recovers data from EEPROM.
    Serial.println("Reading old values from EEPROM");
    EPRM.readFromEEPROM();
    while (1) {
      Serial.println("Read done, please reset");
      delay(100000);
    }
  } else {
    Serial.println("Starting experiment");
    posttime = millis();
  }

  state == STATE_ONLINE;
}


// put your main code here, to run repeatedly:
void loop() {
  // Always call a function to update the state
  // variable.  loop() should be returning quickly.
  // flow control is managed in updateState()
  updateState();

  // Background writing coordinates to grid
  writetogrid();
  
  // This is the basic structure for a FSM  Based on the value
  // of "state" variable, run appropriate code for robot behaviour.
  if ( state == STATE_ONLINE ) {
    //insert line following code here
    LFB();
  } else if ( state == STATE_OFFLINE ) {
    //insert IMU correction code here
    Xposition();
    Serial.println("Robot is off the line");
  } else {
    // You can catch situations where the robot
    // attempts to move into an unknown state.
    Serial.println("Something went wrong");
  }
}

// A function to update the system variables, and to
// cause transitions between system states.
void updateState() {
  //if sensors detect line then:
  //      run line follower
  //if sensors can't detect line then:
  //      run the IMU correction
  LS.LSRead();
  int a;
  int b;
  int c;
  a = LS.elapsed_t[0];
  b = LS.elapsed_t[1];
  c = LS.elapsed_t[2];
  if (a < LINE_THRESHOLD && b < LINE_THRESHOLD && c < LINE_THRESHOLD) {
    state == STATE_OFFLINE;
  } else {
    state == STATE_ONLINE;
  }
  //Serial.println(a);
  //Serial.println(b);
  //Serial.println(c);
  //Serial.println(state);
}

void writetogrid () {
  // Setup our grid in volatile memory
  // as if it had been data collected
  // by the robot.
  unsigned long gridwrite;
  unsigned long gride_t;
  float gridd_t;
  float calc;

  gridwrite = millis();
  gride_t = gridwrite - posttime;
  gridd_t = (float)gride_t;
  
  int y;
  for ( y = 0; y < 20; y++ ) {
    if (gridd_t > 500) {
      EPRM.grid[0][y] = 1.5 * calc * (gridd_t * gridd_t);
      EPRM.grid[1][y] = gridd_t;
      posttime = millis(); 
    }
  }
}

// write this function to have your
// robot beep 5 times, across a total
// of 5 seconds.
void intialisingBeeps() {
  pinMode(6, OUTPUT);
  analogWrite(6, 50);
  delay(1000);
  analogWrite(6, 0);
  analogWrite(6, 50);
  delay(1000);
  analogWrite(6, 0);
  analogWrite(6, 50);
  delay(1000);
  analogWrite(6, 0);
  analogWrite(6, 50);
  delay(1000);
  analogWrite(6, 0);
  analogWrite(6, 50);
  delay(1000);
  analogWrite(6, 0);
}

void LFB() {
  float LFBmeasurement;
  float LFBdemand = 0;
  float Eline;

  LFBmeasurement = LS.LSRead();
  LFPID.PIDreset(255, 0.001, 0.001);                //set PID terms for LFB and reset variables in PID
  Eline = LFPID.PIDupdate(LFBdemand, LFBmeasurement);

  MTR.setLeftMotorPower(MOTOR_PWR - Eline);
  MTR.setRightMotorPower(MOTOR_PWR + Eline);
  IMUX.gyro = 0;
  IMUX.accel = 0;
}

void Xposition() {
  float calc;
  float xposmeasurement;
  float xposdemand;
  float xEline;
  unsigned long now_t;
  unsigned long last_t;
  unsigned long e_t;
  float e_dt;

  now_t = millis();
  e_t = now_t - last_t;

  e_dt = (float)e_t;

  calc = IMUX.iaccel();
  
  xposmeasurement = 1.5 * calc * (e_dt * e_dt);
  
  if (xposmeasurement < 0) {
    xposdemand = -xposmeasurement;
  } else if (xposmeasurement > 0) {
    xposdemand = -xposmeasurement;
  }

  XPPID.PIDreset(255, 0.001, 0.001);                  //set PID terms for x-pos and reset variables in PID
  xEline = XPPID.PIDupdate(xposdemand, xposmeasurement);
  
  MTR.setLeftMotorPower(MOTOR_PWR - xEline);
  MTR.setRightMotorPower(MOTOR_PWR + xEline);
}

void Hangle() {
  float calc;
  float HAmeasurement;
  float HAdemand;
  float hEline;
  unsigned long now_t;
  unsigned long last_t;
  unsigned long e_t;
  float e_dt;

  now_t = millis();
  e_t = now_t - last_t;

  e_dt = (float)e_t;

  calc = IMUX.igyro();
  
  HAmeasurement = calc * e_dt;
  
  if (HAmeasurement < 0) {
    HAdemand = -HAmeasurement;
  } else if (HAmeasurement > 0) {
    HAdemand = -HAmeasurement;
  }

  HPID.PIDreset(255, 0.001, 0.001);                  //set PID terms for HA and reset variables in PID
  hEline = HPID.PIDupdate(HAdemand, HAmeasurement);
  
  MTR.setLeftMotorPower(MOTOR_PWR - hEline);
  MTR.setRightMotorPower(MOTOR_PWR + hEline);
}
