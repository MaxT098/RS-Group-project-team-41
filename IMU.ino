#include <Wire.h>
#include <LSM6.h>
#include <Motors.h>
#include <LineSensors.h>
#include <Others.h>

#define LINE_SENSOR_UPDATE  100
#define MOTOR_UPDATE        2000
#define MOTOR_RUNTIME       500
#define MOTOR_PWR           40
#define TURNING_PWR         500

#define LINE_THRESHOLD      2000
#define DF_TIMEOUT          3000
#define ELINE_THRESHOLD     0.2

// LSM6 is a class.  We create an instance
// of this class called "imu".  We will then
// use imu to access the device by method
// functions and variables inside the class.
LSM6 imu;

float accel;
float gyro;

void setup() {
  
  Motors.initialise();
  LineSensors.initialise();

  // Start the wire library for i2c.
  // Note: do not add this command into
  // a class constructor. It must occur
  // (or be called) from setup().
  Wire.begin();

  // Serial for debug output
  Serial.begin(9600);
  Serial.println("***RESET***");
  delay(1000);

  // Check the IMU initialised ok.
  if (!imu.init() ) {  // no..? :(

    // Since we failed to communicate with the
    // IMU, we put the robot into an infinite
    // while loop and report the error.
    while (1) {
      Serial.println("Failed to detect and initialize IMU!");
      delay(1000);
    }
  }

  // IMU initialise ok!
  // Set the IMU with default settings.
  imu.enableDefault();

} // End of setup()

// Simple loop to report all the readings from
// the accelerometer and gyro
// imu.a = accelerometer
// imu.g = gyro
void loop() {

  // Make a read of the sensor.
  imu.read();

  // Report values
  accel = (imu.a.y * 0.061 / 1000);
  gyro = (imu.g.z * 8.75 / 1000);

  if (gyro < 0) {
    accel = -(accel);
  }

  lineFollowingBehaviour();

  // Short delay to keep things
  // slow enough to observe.
  // There is a limit to how fast you
  // can make i2c readings.
  delay(100);


} // End of loop()

void lineFollowingBehaviour() {
  unsigned long ls_ts;
  unsigned long motor_ts;
  unsigned long elapsed_time;
  float Eline;

  elapsed_time = current_ts - ls_ts;
  if ( elapsed_time > LINE_SENSOR_UPDATE ) {
    LineSensors.AllLineSensorsRead();
    ls_ts = millis();
  }

  elapsed_time = current_ts - motor_ts;
  if ( elapsed_time > MOTOR_UPDATE ) {
    Eline = LineSensors.getLineError();

    //When not detecting a line
    if (LineSensors.elapsed_t[0] > LINE_THRESHOLD && LineSensors.elapsed_t[1] > LINE_THRESHOLD && LineSensors.elapsed_t[2] > LINE_THRESHOLD) {
      Motors.setLeftMotorPower(-TURNING_PWR * gyro);
      Motors.setRightMotorPower(TURNING_PWR * gyro);
    } else if (Eline > ELINE_THRESHOLD) {
      Motors.setLeftMotorPower(-TURNING_PWR * Eline);
      Motors.setRightMotorPower(TURNING_PWR * Eline);
    } else if (Eline < -ELINE_THRESHOLD) {
      Motors.setLeftMotorPower(-TURNING_PWR * Eline);
      Motors.setRightMotorPower(TURNING_PWR * Eline);
    } else {
      Motors.setLeftMotorPower(MOTOR_PWR);
      Motors.setRightMotorPower(MOTOR_PWR);
    }
    motor_ts = millis();
  }
}
