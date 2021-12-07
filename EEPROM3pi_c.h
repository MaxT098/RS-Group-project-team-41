// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _EEPROM3pi_H
#define _EEPROM3pi_H

// Class to contain generic PID algorithm.
class EEPROM3pi_c {
  public:
#include <EEPROM.h>
#define BUTTON_A_PIN  14
#define BUTTON_B_PIN  30

    float grid[2][20];

    // Constructor, must exist.
    EEPROM3pi_c() {

    }
    // writes a 2D grid into the 1D
    // EEPROM.  Note, readFromEEPROM
    // uses a consistent method to
    // get the data back out.
    void writeGridToEEPROM() {
      int x, y, address;

      address = 0;
      for ( y = 0; y < 20; y++ ) {
        for ( x = 0; x < 2; x++ ) {

          // Update will only write to the EEPROM
          // if the value has changed.  This should
          // help the EEPROM to stay working for
          // longer.
          EEPROM.update( address, grid[x][y] );

          address++; // adds 1 to address
        }
      }

    }

    // Serial prints the contents of
    // EEPROM
    void readFromEEPROM() {
      int x, y, address;

      address = 0;
      for ( y = 0; y < 20; y++ ) {
        for ( x = 0; x < 2; x++ ) {

          // Get a bye at address from EEPROM
          float value = EEPROM.read( address );

          // Print as output
          Serial.print( value );
          Serial.print(",");
          address++; // adds 1 to address
        }
        // Newline after print a row
        Serial.print("\n");
      }
      //Done.
      Serial.println("***********\n");
    }
};

#endif
