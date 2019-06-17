/*#include "Ultrasonic.h"
const float Kp = 0.025, Ki = 0., Kd = 0.01; // Regelaarparameters
const float sp = 30;       // setpoint = 30 cm

float error_oud, D_error;
float I_error = 0;
float error = 0;


float pwmLv, pwmLa, pwmRa, pwmRv;

bool richting;


    
const float aLv = 993.44, aRv = 1007.9; // vooruit bewegend lineair
const float aLa = -2286.2, bLa = 1234.9, aRa = -1964.7,  bRa = 1169.1; // achteruit bewegend polynoom
//----------------------------------------------------------------------------/
// Houd rekening met de maximale stuwkracht voor zowel linker als rechter motor en met
// vooruit en achteruit.
//----------------------------------------------------------------------------/
const float FmaxL = 0.1722, FmaxR = 0.1678, FminL = -0.2278, FminR = -0.2295;
//---------------------------------------------------------------------------/


const float Fmax = min(FmaxL, FmaxR), Fmin = max(FminL, FminR);           // N
float F, Fclipped;

int maxpwm = 170;

const int motor1Lpen = 5;
const int motor1Rpen = 6;
const int motor2Lpen = 9;
const int motor2Rpen = 3;

int Round(float myfloat)
{
  double integral;
  float fraction = (float)modf(myfloat, &integral);
 
  if (fraction >= 0.5)
    integral += 1;
  if (fraction <= -0.5)
    integral -= 1;
 
  return (int)integral;
}

const int usTrigPen = 12;
const int usEchoPen = 13;
Ultrasonic ultrasonic(usTrigPen, usEchoPen);


void mm300_regelaar(float dt, int &PWMLv, int &PWMLa, int &PWMRv, int &PWMRa){
  
  float s = ultrasonic.Ranging(CM);
  if(s > 200){
    s = 200;
  }

  //P-regelaar
  error = (sp - s)*0.1;
  
  //D-regelaar
  D_error = (error - error_oud) / dt;

  //I-regelaar
  I_error = I_error + (error*dt);
  
  F = error * Kp + I_error * Ki + D_error * Kd;

  error_oud = error;
/*
  float Fv = C / error;
  if(error > 0 ){
    float Ftot = F - Fv;
    if(Ftot < 0) Ftot = -Ftot;
  }

  Fclipped = max(min(F, Fmax), Fmin); // Fmin < F < +Fmax

  // Omrekening Fclipped naar pwm-signaal
  if (Fclipped > 0) {                 // Als de stuwkracht vooruit gericht is dan..
    pwmLv = aLv * Fclipped;
    pwmRv = aRv * Fclipped;
    //pwmLa = (maxpwm / FmaxL) * Fclipped;
    //pwmRa = (maxpwm / FmaxR) * Fclipped;
    pwmLa = 0;
    pwmRa = 0;
    richting = true;

  } else {                            // ..anders..
      pwmLa = -(aLa * Fclipped * Fclipped + bLa * Fclipped);
      pwmRa = -(aRa * Fclipped * Fclipped + bLa * Fclipped);
      //pwmLv = (maxpwm / FminL) * Fclipped;
      //pwmRv = (maxpwm / FminR) * Fclipped;
      pwmLv = 0;
      pwmRv = 0;
      richting = false;
  }
  PWMLv = max(min(Round(pwmLv), 170), 0);
  PWMRv = max(min(Round(pwmRv), 170), 0);
  PWMLa = max(min(Round(pwmLa), 170), 0);
  PWMRa = max(min(Round(pwmRa), 170), 0);
  /*Serial.print("\t\tPWMRv: ");
  Serial.print(pwmRv);
  Serial.print("\t\tPWMLv: ");
  Serial.print(pwmLv);
  Serial.print("\t\tPWMRa: ");
  Serial.print(pwmRa);
  Serial.print("\t\tPWMLa: ");
  Serial.print(pwmLa);
  //Serial.print("\t\t");
  Serial.print(s);
  Serial.print("\t\t");
  Serial.print(Fclipped);
  Serial.print("\t\t");
}
