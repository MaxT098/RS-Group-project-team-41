#ifndef _LINESENSORS_H
#define _LINESENSORS_H

class LineSensors_c {
  public:
#define EMIT_PIN          11
#define LS_LEFT_IN_PIN    18
#define LS_CENTRE_IN_PIN  20
#define LS_RIGHT_IN_PIN   21
#define tout              2500
#define NB_LS_PINS        3

#include <math.h>

    int elapsed_t[NB_LS_PINS] = {};

    LineSensors_c () {

    }

    void initialise() {
      pinMode(EMIT_PIN, OUTPUT);
      pinMode(LS_LEFT_IN_PIN, INPUT);
      pinMode(LS_CENTRE_IN_PIN, INPUT);
      pinMode(LS_RIGHT_IN_PIN, INPUT);
      digitalWrite(EMIT_PIN, HIGH);
    }

    float LSRead() {
      unsigned long start_time;
      unsigned long end_time;
      unsigned long d_t;
      bool done = false;
      float Lreading;
      float Creading;
      float Rreading;
      float Lnorm;
      float Cnorm;
      float Rnorm;
      float Lweight;
      float Rweight;
      float measurement;

      int ls_pin[NB_LS_PINS] = { LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN };
      unsigned long end_t[NB_LS_PINS] = {};

      pinMode(LS_LEFT_IN_PIN, OUTPUT);
      pinMode(LS_CENTRE_IN_PIN, OUTPUT);
      pinMode(LS_RIGHT_IN_PIN, OUTPUT);
      digitalWrite(LS_LEFT_IN_PIN, HIGH);
      digitalWrite(LS_CENTRE_IN_PIN, HIGH);
      digitalWrite(LS_RIGHT_IN_PIN, HIGH);
      delayMicroseconds(10);
      pinMode(LS_LEFT_IN_PIN, INPUT);
      pinMode(LS_CENTRE_IN_PIN, INPUT);
      pinMode(LS_RIGHT_IN_PIN, INPUT);
      start_time = micros();

      while (done == false) {
        int i;
        for (i = 0; i < NB_LS_PINS; i++) {
          if (digitalRead(ls_pin[i]) == LOW && end_t[i] == 0) {
            end_t[i] = micros();
          }
        }
        if (digitalRead(ls_pin[0]) == digitalRead(ls_pin[1]) == digitalRead(ls_pin[2]) == LOW && end_t[0] != 0 && end_t[1] != 0 && end_t[2] != 0) {
          for (i = 0; i < NB_LS_PINS; i++) {
            if (elapsed_t[i] <= tout) {
              elapsed_t[i] = end_t[i] - start_time;
            } else {
              elapsed_t[i] = tout;
            }
          }
          done = true;
        }
      }

      Lreading = elapsed_t[0];
      Creading = elapsed_t[1];
      Rreading = elapsed_t[2];

      //      Serial.print(Lreading); Serial.print(" ");
      //      Serial.print(Creading); Serial.print(" ");
      //      Serial.print(Rreading); Serial.print(" ");
      //      Serial.println();

      Lnorm = Lreading / (Lreading + Creading + Rreading);    //The measurement
      Cnorm = Creading / (Lreading + Creading + Rreading);
      Rnorm = Rreading / (Lreading + Creading + Rreading);

      Lweight = Lnorm + (Cnorm * 0.25);
      Rweight = Rnorm + (Cnorm * 0.25);
      measurement = 0 - (Lweight - Rweight);

      return measurement;
    }
};

#endif
