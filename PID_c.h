// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:

    // Constructor, must exist.
    PID_c() {

    }

    float PIDupdate (float demand, float measurement) {
      float inter;
      float Pterm;              //The P-term
      float Iterm;              //The I-term
      float Dterm;              //The D-term
      float feedback;           //The feedback signal
      float error;              //The error
      float preverror;          //The previous error
      unsigned long iterm_last;
      unsigned long iterm_now;
      unsigned long iterm_elapsed;
      float iterm_dt;

      error = demand - measurement;

      Pterm = Kp * error;                                //note that the pterm is multiplied by proportional gain#

      iterm_now = millis();
      iterm_elapsed = iterm_now - iterm_last;

      iterm_dt = (float)iterm_elapsed;

      if (iterm_elapsed == 0) return feedback;

      inter = inter + (error * iterm_dt);

      Iterm = Ki * inter;

      Dterm = Kd * (error - preverror) / iterm_dt;   //calculation of the D-term

      preverror = error;
      iterm_last = millis();
      feedback = Pterm;// + Iterm;// + Dterm;

      //Serial.print(Pterm);
      //Serial.print(",");
      //Serial.print(Iterm);
      //Serial.print(",");
      //Serial.print(Dterm);
      //Serial.print("\n");

      return feedback;
    }
};



#endif
