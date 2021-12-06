#ifndef _MOTORS_H
#define _MOTORS_H

class Motors_c {
  public:
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

    Motors_c() {

    }

    void initialise() {
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);
      digitalWrite(L_DIR_PIN, LOW);
      digitalWrite(R_DIR_PIN, LOW);
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
    }

    void setLeftMotorPower(float pwm) {
      float pwma;
      if (-255 <= pwm <= 255) {
        float pwma = abs(pwm);
        if (pwm >= 0) {
          digitalWrite(L_DIR_PIN, LOW);
          analogWrite(L_PWM_PIN, pwma);
        } else {
          digitalWrite(L_DIR_PIN, HIGH);
          analogWrite(L_PWM_PIN, pwma);
        }
      } else {
        Serial.println("LEFT motor power value input ERROR");
      }
    }

    void setRightMotorPower(float pwm) {
      float pwma;
      if (-255 <= pwm <= 255) {
        float pwma = abs(pwm);
        if (pwm >= 0) {
          digitalWrite(R_DIR_PIN, LOW);
          analogWrite(R_PWM_PIN, pwma);
        } else {
          digitalWrite(R_DIR_PIN, HIGH);
          analogWrite(R_PWM_PIN, pwma);
        }
      } else {
        Serial.println("RIGHT motor power value input ERROR");
      }
    }
};

#endif
