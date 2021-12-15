#include "Motors_c.h"               // Packages
#include "LineSensors_c.h"
#include "Imux_c.h"
#include "PID_c.h"
#include <USBCore.h>


#define LINE_SENSOR_UPDATE  100     // Update timings

#define MOTOR_PWR           30      // Motor base power

#define LINE_THRESHOLD      1200    // A threshold for detecting whether there is a line or not, lower than this is no line.

#define UPDATE_MS           100     // Data saving interval

#define BUTTON_A_PIN        14      // Push buttons connected to:
#define BUTTON_B_PIN        30

#define RESULTS_NUM         80      // Results grid size

Motors_c MTR;                       // Create instances of classes
LineSensors_c LS;
Imux_c IMUX;
PID_c LFPID;
PID_c XPPID;
PID_c HPID;

u8 USB_SendSpace(u8 ep);            //USB stuff
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#define STATE_OFFLINE   0           // States for the state machine
#define STATE_ONLINE    1
#define STATE_INIT      2           // Paul: please see comments in updateState()
#define STATE_FINISH    3           // and Xposition() and LFB()


int state;                          // This will hold which state your robot is in

unsigned long posttime;             // Timer for background x-position mapping

int grid_address;                   // int for storing grid address

unsigned long last_t;               // Paul: extracted from Xposition(), see function
                                    //       for more comments.

float xpos_lpf = 0.0;               // Paul: accelerometer has high freq noise, so
                                    // trying a crude low-pass filter. See Xposition();

float current_value;                // xaccel variables that needs to be saved globally
float saved_value;                  // to facilitate reset every time it returns to line
float corr;

float results[RESULTS_NUM];

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

  // Serial initialisation
  Serial.begin(9600);
  delay(1000);
  Serial.println("***Setup Complete***");

  // Wait for a button press to decide what to do, gate for start of experiment2
  int a;

  a = HIGH;

  do {
    Serial.println("A = start experiment");
    a = digitalRead(BUTTON_A_PIN);
  } while ( a == HIGH );

  if ( a == LOW ) {
    Serial.println("Starting experiment");
    grid_address = 0;                   //sets grid address to 0
    posttime = millis();
  }

  state = STATE_INIT;  // Paul: start in an init state, so that we can then initialise into a correct state properly.
}


// put your main code here, to run repeatedly:
void loop() {

  updateState();

  // Background writing coordinates to grid
  writetogrid();

  Serial.println(grid_address);

  // This is the basic structure for a FSM  Based on the value
  // of "state" variable, run appropriate code for robot behaviour.
  if ( state == STATE_ONLINE ) {

    current_value = 0;
    saved_value = 0;
    corr = 0;
    LFB();
    
  } else if ( state == STATE_OFFLINE ) {
    
    Xposition();
    
  } else if ( state == STATE_FINISH ) {
    while (1) {
      MTR.setLeftMotorPower(0);
      MTR.setRightMotorPower(0);
      //reportResultsOverSerial();
    }
  } else {
    Serial.println("Something went wrong");
  }
}

// A function to update the system variables, and to
// cause transitions between system states.
void updateState() {
  LS.LSRead();
  int a;
  int b;
  int c;
  a = LS.elapsed_t[0];
  b = LS.elapsed_t[1];
  c = LS.elapsed_t[2];

  if (a < LINE_THRESHOLD && b < LINE_THRESHOLD && c < LINE_THRESHOLD && state != STATE_OFFLINE) {

    state = STATE_OFFLINE;

    last_t = millis();

    XPPID.PIDreset(0.4, 0.001, 0.001);                  //set PID terms for x-pos and reset variables in PID

  } else if ( (a >= LINE_THRESHOLD || b >= LINE_THRESHOLD || c >= LINE_THRESHOLD ) && state != STATE_ONLINE ) {

    state = STATE_ONLINE;

    LFPID.PIDreset(50, 0.001, 0.001);                   //set PID terms for LFB and reset variables in PID
  } //else if (grid_address = RESULTS_NUM /*&& (a < LINE_THRESHOLD && b < LINE_THRESHOLD && c < LINE_THRESHOLD) && state != STATE_FINISH*/) {
    //state = STATE_FINISH;
  //}
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
  xpos = 1.5 * calc * ((UPDATE_MS / 1000) ^ 2);

  if ( millis() - posttime > UPDATE_MS && grid_address < RESULTS_NUM) {
    posttime = millis();

    results[grid_address] = xpos;      //change this to write to a grid

    grid_address += 1;
  }
}

void reportResultsOverSerial() {
  // Print millis for debug so we can
  // validate this is working in real
  // time, and not glitched somehow
  if ( SERIAL_ACTIVE ) Serial.println( "Position: " );
  delay(1);

  // Loop through array to print all
  // results collected

  int i;
  for ( i = 0; i < RESULTS_NUM; i++ ) {
    // Comma seperated values, to 2 decimal places
    if ( SERIAL_ACTIVE ) Serial.println( results[i], 2 );
    delay(1);
  }
  if ( SERIAL_ACTIVE ) Serial.println( "---End of Results ---\n" );
}

void LFB() {
  float LFBmeasurement;
  float LFBdemand = 0;
  float Eline;

  LFBmeasurement = LS.LSRead();
          
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
  float xposmeasurement;    //remnants?
  float xposdemand;
  float xEline;
  unsigned long now_t;

  unsigned long e_t;
  float e_dt;

  now_t = millis();
  e_t = now_t - last_t;

  last_t = now_t;

  if ( e_t == 0 ) return;

  e_dt = (float)e_t;

  xposmeasurement = IMUX.iaccel();   //change this term to switch correction modes! Seems like need to adjust motor input direction for gyro

  float alpha = 0.9;
  
  // Paul: with alpha=0.9, this will mean we "trust" old
  //       readings more (or reject new readings).
  
  xpos_lpf = ( xpos_lpf * alpha ) + ( xposmeasurement * (1 - alpha) );

  current_value = xpos_lpf;

  if (current_value > 0 && saved_value >= 0 && current_value > saved_value) {       //fix this I think it is the problem why when we move it right it still increases the negative value
    saved_value = current_value;
    corr = saved_value;
  } else if (current_value < 0 && saved_value <= 0 && current_value < saved_value) {
    saved_value = current_value;
    corr = saved_value;
  } else {
    corr = saved_value;
  }

  if (corr <= 0) {
    MTR.setLeftMotorPower(MOTOR_PWR - (corr * 2.5));
    MTR.setRightMotorPower(MOTOR_PWR - corr);
  } else {
    MTR.setLeftMotorPower(MOTOR_PWR + corr);
    MTR.setRightMotorPower(MOTOR_PWR + (corr * 2.5));
  }
  
  // (Probably don't need this now because we are using a different correction algorithm, see above)
  // Paul: Just above, we set the xposdemand to equal the inverse
  //       of the measurement.  Is this what you want to do?
  //      I get the feeling that we want the demand to be 0.
  //      Therefore,the robot tries to minimise any velocity
  //      in either direction down to 0.
  //xEline = XPPID.PIDupdate(xposdemand, xposmeasurement);
  //xEline = XPPID.PIDupdate(0.0, xpos_lpf);

  // Paul: debugging by printing out feedback from imu PID.

  //Serial.println(xposmeasurement);
  //Serial.print("C =");
  //Serial.println(current_value);
  //Serial.print("S =");
  //Serial.println(saved_value);
  //Serial.print("Corr =");
  //Serial.println(corr);

}
