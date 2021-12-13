#include "Motors_c.h"               // Packages
#include "LineSensors_c.h"
#include "Imux_c.h"
#include "EEPROM3pi_c.h"
#include "PID_c.h"
#include <EEPROM.h>

#define LINE_SENSOR_UPDATE  100     // Update timings

#define MOTOR_PWR           30      // Motor base power

#define LINE_THRESHOLD      1200    // A threshold for detecting whether there is a line or not, lower than this is no line.

#define UPDATE_MS   500             // Data saving interval

#define BUTTON_A_PIN  14            // Push buttons connected to:
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
#define STATE_INIT      2           // Paul: please see comments in updateState()
                                    // and Xposition() and LFB()

int state;                          // This will hold which state your robot is in

unsigned long posttime;             // Timer for background x-position mapping
int eeprom_address;                 // Int for storing array address

unsigned long last_t;               // Paul: extracted from Xposition(), see function
                                    //       for more comments.

float xpos_lpf = 0.0;               // Paul: accelerometer has high freq noise, so
                                    // trying a crude low-pass filter. See Xposition();

float current_value;                //xaccel variables that needs to be saved globally
float saved_value;                  //to facilitate reset every time it returns to line
float corr;

// put your setup code here, to run once:
void setup() {

  MTR.initialise();                 // Standard initialisations for the Motors and line sensor
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

  eeprom_address = 0;                   //sets eeprom address to 0

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



  state = STATE_INIT;  // Paul: start in an init state, so that
                       //we can then initialise into a
                       //correct state properly.
}


// put your main code here, to run repeatedly:
void loop() {
  // Always call a function to update the state
  // variable.  loop() should be returning quickly.
  // flow control is managed in updateState()
  updateState();

  // Background writing coordinates to grid
  //writetogrid();

  // This is the basic structure for a FSM  Based on the value
  // of "state" variable, run appropriate code for robot behaviour.
  if ( state == STATE_ONLINE ) {
    //insert line following code here
    current_value = 0;
    saved_value = 0;
    corr = 0;
    LFB();
  } else if ( state == STATE_OFFLINE ) {
    //insert IMU correction code here
    Xposition();
    //Serial.println("Robot is off the line");
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

  // Paul: because we need to set some variables only at the
  //       time we transition state, I've added in a condition
  //       to check which state we are already in.  This means
  //       that the state change code will only run when it is
  //       necessary to change state.  Without this, the state
  //       is continually set when updateState() is called, and
  //       we can't do one-time initialisations, such as to PID
  //       gains.
  //       To make sure the FSM initialises well, I've added an
  //       INIT state, so at least one of the following will then
  //       be true (because the robot will not start in STATE_ONLINE
  //       or STATE_OFFLINE.

  if (a < LINE_THRESHOLD && b < LINE_THRESHOLD && c < LINE_THRESHOLD && state != STATE_OFFLINE) {

    state = STATE_OFFLINE;


    // Paul: because we move into the new state STATE_OFFLINE,
    //       we need to update the global last_t to be the time
    //       now, so that we don't have a huge dt value in the
    //       function.  Huge because, last_t will have the value
    //       last saved before it came back to this state.
    //       There is a chance we may get last_t = 0, which could
    //       cause an error. I think this is fixed in XPosition()
    last_t = millis();

    // Paul: as per Xposition() comments, I think you want to
    //       perform this reset only once, when you move into
    //       this new state.  Otherwise your PID will never
    //       calculate dt properly (iterm_last will keep being
    //       updated to the current time).
    XPPID.PIDreset(0.4, 0.001, 0.001);                  //set PID terms for x-pos and reset variables in PID

  } else if ( (a >= LINE_THRESHOLD || b >= LINE_THRESHOLD || c >= LINE_THRESHOLD ) && state != STATE_ONLINE ) {
    // Paul: check we are not already in this state,
    //       so we only set the state flag once.
    //       We now can't just use "else", we need to check
    //       that we are in an online condition, and that we
    //       are not already in state = online.
    //       We use || (or) because if any one sensor is on
    //       the line, we qualify as state = online

    // Paul: bug here, this is a condition equivalence, not
    //       assignment.
    state = STATE_ONLINE;


    // Paul: as per LFB() comments, I think you want to
    //       perform this reset only once, when you move into
    //       this new state.  Otherwise your PID will never
    //       calculate dt properly (iterm_last will keep being
    //       updated to the current time).
    //       Once I fully debugged your code execution, these
    //       PID gains were no longer sensible.  I've retuned
    //       them for my robot.
    LFPID.PIDreset(50, 0.001, 0.00);
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
  float calc;
  float xpos;

  calc = IMUX.iaccel();

  // This if statement runs every UPDATE_MS
  // which was set to 1000ms (1s)
  if ( millis() - posttime > UPDATE_MS ) {
    posttime = millis();
    xpos = 1.5 * calc * (UPDATE_MS * UPDATE_MS);

    //EEPROM.update( eeprom_address, xpos);

    eeprom_address++;
  }

  if ( eeprom_address > 20 ) {

    // An infinite while loop to stop
    // the robot.
    while (1) {
      MTR.setLeftMotorPower(0);
      MTR.setRightMotorPower(0);
      // zero motor pwm.
      // indicate finished.
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

  // Paul:  I am not sure you want to call reset every time
  //        this function runs.  This will cause your PID
  //        iterm_last to be reset, meaning your PID dt
  //        calculation will be wrong.
  //        Instead, I think you want to make this call once
  //        the first time you transition into the state which
  //        calls this function, LFB(), which is
  //        state == STATE_ONLINE (I think).
  //        Please see addition in updateState()
  //LFPID.PIDreset(255, 0.001, 0.001);                //set PID terms for LFB and reset variables in PID
  Eline = LFPID.PIDupdate(LFBdemand, LFBmeasurement);

  // Paul: debugging by looking at feedback value
  //Serial.println( Eline );
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
  //unsigned long last_t; // Paul: Because last_t is declared here inside
  //       this function, it gets deleted every time
  //       the function returns.  We need last_t to
  //       keep its value between function calls.
  //       So i've commented this out and added it
  //       to global variables.
  unsigned long e_t;
  float e_dt;

  now_t = millis();
  e_t = now_t - last_t;

  // Paul: I think you forgot to update last_t
  //       to now_t here, which means e_t was
  //       not the correct difference in time
  last_t = now_t;

  // If we have had 0 milliseconds elapse, we
  // could get a calculation error, so we just
  // return so we come back next time with
  // a last_t > 0
  if ( e_t == 0 ) return;

  e_dt = (float)e_t;

  xposmeasurement = IMUX.iaccel();

  // Paul: I am not sure why *1.5, but I'll leave
  //       this in.  Removing it will effect PID
  //       gains.
  //xposmeasurement = 1.5 * calc * (e_dt * e_dt);

  float alpha = 0.9;
  // Paul: with alpha=0.9, this will mean we "trust" old
  //       readings more (or reject new readings).
  xpos_lpf = ( xpos_lpf * alpha ) + ( xposmeasurement * (1 - alpha) );

  current_value = xpos_lpf;

  if (current_value > 0 && saved_value >= 0 && current_value > saved_value) {       //fix this I think it is the problem why when we move it right it still increases the negative value
    saved_value = current_value;
    corr = saved_value * 2;
  } else if (current_value < 0 && saved_value <= 0 && current_value < saved_value) {
    saved_value = current_value;
    corr = saved_value;
  } else {
    corr = saved_value;
  }

  // Paul: I think this if statement is attempting
  //       to invert the direction of the measurement?
  //       The same can be achieved by *-1 I think?
  //if (xposmeasurement < 0) {
  //  xposdemand = -xposmeasurement;
  //} else if (xposmeasurement > 0) {
  //  xposdemand = -xposmeasurement;
  //}

  // Paul: i set your demand to 0 instead
  //xposdemand = xposmeasurement * -1.0;

  if (corr <= 0) {
    MTR.setLeftMotorPower(MOTOR_PWR - (corr * 2));
    MTR.setRightMotorPower(MOTOR_PWR - corr);
  } else {
    MTR.setLeftMotorPower(MOTOR_PWR + corr);
    MTR.setRightMotorPower(MOTOR_PWR + (corr * 2));
  }

  // Paul:  I am not sure you want to call reset every time
  //        this function runs.  This will cause your PID
  //        iterm_last to be reset, meaning your PID dt
  //        calculation will be wrong.
  //        Instead, I think you want to make this call once
  //        the first time you transition into the state which
  //        calls this function, Xposition(), which is
  //        state == STATE_OFFLINE (I think).
  //        Please see addition in updateState()
  //XPPID.PIDreset(255, 0.001, 0.001);                  //set PID terms for x-pos and reset variables in PID

  // Paul: Just above, we set the xposdemand to equal the inverse
  //       of the measurement.  Is this what you want to do?
  //      I get the feeling that we want the demand to be 0.
  //      Therefore,the robot tries to minimise any velocity
  //      in either direction down to 0.
  //xEline = XPPID.PIDupdate(xposdemand, xposmeasurement);
  //xEline = XPPID.PIDupdate(0.0, xpos_lpf);

  // Paul: debugging by printing out feedback from imu PID.
  //Serial.print("C =");
  //Serial.println(current_value);
  //Serial.print("S =");
  //Serial.println(saved_value);
  //Serial.print("Corr =");
  //Serial.println(corr);

}


// Paul: At the moment, it looks like this functions isn't called.
//       However, it also now uses the global last_t, so be careful
//       that this doesn't cause a bug. You might want to create to
//       different last_t, one associated to each function.  This
//       will avoid one function corrupting the others dt calculation.
void Hangle() {
  float calc;
  float HAmeasurement;
  float HAdemand;
  float hEline;
  unsigned long now_t;
  //unsigned long last_t; // Paul: same comment as Xposition()
  unsigned long e_t;
  float e_dt;

  now_t = millis();
  e_t = now_t - last_t;

  // If we have had 0 milliseconds elapse, we
  // could get a calculation error, so we just
  // return so we come back next time with
  // a last_t > 0
  if ( e_t == 0 ) return;

  // Paul: I think you forgot to update last_t
  //       to now_t here, which means e_t was
  //       not the correct difference in time
  last_t = now_t;

  e_dt = (float)e_t;

  calc = IMUX.igyro();

  HAmeasurement = calc * e_dt;

  if (HAmeasurement < 0) {
    HAdemand = -HAmeasurement;
  } else if (HAmeasurement > 0) {
    HAdemand = -HAmeasurement;
  }

  // Paul: Note: you'd need to move reset to the updateState
  //       function just like Xposition and LFB
  HPID.PIDreset(255, 0.001, 0.001);                  //set PID terms for HA and reset variables in PID
  hEline = HPID.PIDupdate(HAdemand, HAmeasurement);

  MTR.setLeftMotorPower(MOTOR_PWR - hEline);
  MTR.setRightMotorPower(MOTOR_PWR + hEline);
}
