// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:

    float Kp;
    float Ki;
    float Kd;
    float inter;
    float Pterm;              //The P-term
    float Iterm;              //The I-term
    float Dterm;              //The D-term
    float feedback;           //The feedback signal
    float preverror;          //The previous error
    unsigned long iterm_last;

    // Constructor, must exist.
    PID_c() {

    }
    
    void PIDreset (float a, float b, float c) {
      
      inter = 0;
      Pterm = 0;
      Iterm = 0;
      Dterm = 0;
      feedback = 0;
      preverror = 0;       
      Kp = a;
      Ki = b;
      Kd = c;
      iterm_last = millis();
    }

    float PIDupdate (float demand, float measurement) {
      float error;                                       //The error
      unsigned long iterm_now;
      unsigned long iterm_elapsed;
      float iterm_dt;
      
      iterm_now = millis();
      iterm_elapsed = iterm_now - iterm_last;
      iterm_last = millis();
      iterm_dt = (float)iterm_elapsed;
      
      if (iterm_elapsed == 0) return feedback;
      
      error = demand - measurement;

      Pterm = Kp * error;                                 //note that the pterm is multiplied by proportional gain#

      inter = inter + (error * iterm_dt);                 //calculation of the I-term

      Iterm = Ki * inter;

      Dterm = Kd * (error - preverror) / iterm_dt;        //calculation of the D-term

      preverror = error;
      
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
