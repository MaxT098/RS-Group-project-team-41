#ifndef _IMUX_H
#define _IMUX_H
#include <Wire.h>
#include <LSM6.h>

#define gyrofilter -0.634166667
#define accelfilter 0.060966667

class Imux_c {
  public:

    float gyro;
    float accel;

    LSM6 imu;

    Imux_c() {

    }

    void initialise() {
      // Start the wire library for i2c.
      // Note: do not add this command into
      // a class constructor. It must occur
      // (or be called) from setup().
      Wire.begin();
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
    }

    float igyro () {
      imu.read();
      gyro = (imu.g.z * 8.75 / 1000 - gyrofilter);
      return gyro;
    }

    float iaccel () {
      imu.read();
      accel = (imu.a.y * 0.061 / 1000 * 9.81 - accelfilter);
      return accel;
    }
};
#endif
