/*
    Implementatie van PD-regelaar om hovercraft 30 cm van de muur te laten stoppen
    met begrenzer dmv Fmax.
    Benodigd:
    - lineaire benadering van de stuwkrachten vooruit en achteruit per motor
      dwz de a en b van de vergelijking pwm = a.F + b
    - De maximale stuwkrachten vooruit (Fmax) en achteruit (Fmin) per motor
*/

//Libraries
#include "Ultrasonic.h"       // HC-SR04

/****************************************************************************/
// AANPASSEN!!
// Instellingen regelaar
const float Kp = 0.01, Kd = 1., Ki = 0.; // Regelaarparameters
/****************************************************************************/
float s;
const float sp = 30;       // setpoint = 30 cm

float error_oud, D_error;
float I_error = 0;
float error = 0;
const float m = 1;      // kg


/****************************************************************************/
const int usTrigPen = 12;
const int usEchoPen = 13;
//Define pins ultrasonic(trig,echo)
Ultrasonic ultrasonic(usTrigPen, usEchoPen);

/****************************************************************************/
float pwmLv, pwmLa,pwmRa,pwmRv;

bool richting;


// Lineaire benadering (y = a.x + b) van de omrekening van stuwkracht naar pwm-signaal
// met y = pwm en x = F. Er zijn vier benaderingen: voor zowel de linker als de rechter
// motor, en voor vooruit en achteruit bewegen.
/****************************************************************************/
// AANPASSEN!!
// links vooruit: pwm= 1909,2*(F)-524,83
// rechts vooruit: pwm= 2114,7*(F)-590,52
// links achteruit: pwm= 751,31*(f)-157,52
// rechts achteruit: pwm= 737,27*(F)-153,19
    
const float aLv = 1909.2, bLv = -524.83, aRv = 2114.7,  bRv = -590.52; // vooruit bewegend
const float aLa = 751.31, bLa = -157.52, aRa = 737.27,  bRa = -153.19; // achteruit bewegend
/****************************************************************************/
// Houd rekening met de maximale stuwkracht voor zowel linker als rechter motor en met
// vooruit en achteruit.
/****************************************************************************/
// AANPASSEN!!
const float FmaxL = 0.3690, FmaxR = 0.3647, FminL = -0.4590, FminR = -0.4635;
/****************************************************************************/


const float Fmax = min(FmaxL, FmaxR), Fmin = max(FminL, FminR);           // N
float F, Fclipped;            // clippen = F begrenzen tussen -Fmax en +Fmax

long current_time = millis();
float dt;
int x = 1000;
//int x_as = 0;
//const int x_max = 500;      // aantal integratiestappen

int maxpwm = 170;


const int motor1Lpen = 5;
const int motor1Rpen = 6;
const int motor2Lpen = 9;
const int motor2Rpen = 3;

void setup() {

  pinMode(7, OUTPUT);
  pinMode(motor1Lpen, OUTPUT);
  pinMode(motor1Rpen, OUTPUT);
  pinMode(motor2Lpen, OUTPUT);
  pinMode(motor2Rpen, OUTPUT);
          
   
  Serial.begin(9600);
  //s = 0;
  error = sp - s;
  
  //D-regelaar
  D_error = (error - error_oud) / dt;
  I_error = I_error + (error*dt);
  F = error * Kp + I_error * Ki + D_error * Kd;

  error_oud = error;
}

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

int limit_pwm(float pwm){

  if(pwm > 170) pwm = 170;
  if(pwm < 0) pwm = 0;
  return (Round(pwm));
}

void loop() {

  digitalWrite(7, HIGH); 
  dt = (current_time - millis())*0.001;
  current_time = millis();
  
  s = ultrasonic.Ranging(CM); //Use 'CM' for centimeters or 'INC' for inches
  //Serial.print(s);
  //s = s + Fclipped;
  
  //P-regelaar
  error = sp - s;
  
  //D-regelaar
  D_error = (error - error_oud) / dt;
  I_error = I_error + (error*dt);
  F = error * Kp + I_error * Ki + D_error * Kd;

  error_oud = error;

  Fclipped = max(min(F, Fmax), Fmin); // Fmin < F < +Fmax

  // Omrekening Fclipped naar pwm-signaal
  if (Fclipped > 0) {                 // Als de stuwkracht vooruit gericht is dan..
    //pwmLv = aLv * Fclipped + bLv;
    //pwmRv = aRv * Fclipped + bRv;
    pwmLa = (maxpwm / FmaxL) * Fclipped;
    pwmRa = (maxpwm / FmaxR) * Fclipped;
    pwmLv = 0;
    pwmRv = 0;
    analogWrite(motor1Lpen, pwmLv);
    analogWrite(motor2Lpen, pwmLa);
    richting = true;
  } else {                            // ..anders..
      //pwmLa = -(aLa * Fclipped + bLa);
      //pwmRa = -(aRa * Fclipped + bLa);
      pwmLv = (maxpwm / FminL) * Fclipped;
      pwmRv = (maxpwm / FminR) * Fclipped;
      pwmLa = 0;
      pwmRa = 0;
      analogWrite(motor1Rpen, pwmRv);
      analogWrite(motor2Rpen, pwmRa); 
      richting = false;
  }
 
  int PWMLv = limit_pwm(pwmLv);
  int PWMRv = limit_pwm(pwmRv);
  int PWMLa = limit_pwm(pwmLa);
  int PWMRa = limit_pwm(pwmRa);
  analogWrite(motor1Lpen, limit_pwm(pwmLv));
  analogWrite(motor1Rpen, limit_pwm(pwmLa));
  analogWrite(motor2Lpen, limit_pwm(pwmRv));
  analogWrite(motor2Rpen, limit_pwm(pwmRa)); 

  //analogWrite(motor2Lpen, 170);
  //analogWrite(motor1Lpen, 170);
  if(x > 0){
    //x--;
    if (s < 1000)Serial.print(s / 100);
    Serial.print("\t\t");
    Serial.print(pwmRv);
    Serial.print("\t\t");
    Serial.print(pwmLv);
    Serial.print("\t\t");
    Serial.print("\t\t");
    Serial.print(pwmRa);
    Serial.print("\t\t");
    Serial.print(pwmLa);
    Serial.print("\t\t");
    //Serial.println(Fclipped);
    Serial.println("");
  }

}
