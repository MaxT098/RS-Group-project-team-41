#ifndef _OTHER_H
#define _OTHER_H

class Other {
  public:

    Other() {

    }

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

    void LED() {
      pinMode(13, OUTPUT);
      digitalWrite(13, HIGH);
      delay(500);
      digitalWrite(13, LOW);
    }
};

#endif
