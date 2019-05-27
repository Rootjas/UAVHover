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
const float Kp = 1., Kd = 1.; // Regelaarparameters
/****************************************************************************/
float s;
const float sp = 0.300;       // setpoint = 30 cm

float error, error_oud, d_error;
const float m = 1;      // kg


/****************************************************************************/
const int usTrigPen = 8;
const int usEchoPen = 4;
//Define pins ultrasonic(trig,echo)
Ultrasonic ultrasonic(usTrigPen, usEchoPen);

const int motor1Lpen = 5;
const int motor1Rpen = 6;
const int motor2Lpen = 3;
const int motor2Rpen = 9;
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

long t_nw = millis();
long t_oud;
float dt;
const long cyclustijd = 50; // ms

//int x_as = 0;
//const int x_max = 500;      // aantal integratiestappen

void setup() {
  Serial.begin(9600);
}

void loop() {

  s = ultrasonic.Ranging(CM); //Use 'CM' for centimeters or 'INC' for inches
  Serial.println(s);
/*
  // Regelaar
  error_oud = error;
  error = sp - s;
  d_error = error - error_oud;
  F = error * Kp + d_error / dt * Kd;

  Fclipped = max(min(F, Fmax), Fmin); // Fmin < F < +Fmax

  // Omrekening Fclipped naar pwm-signaal
  if (Fclipped > 0) {                 // Als de stuwkracht vooruit gericht is dan..
    pwmLv = aLv * Fclipped + bLv;
    pwmRv = aRv * Fclipped + bRv;
    pwmLa = 0;
    pwmRa = 0;
    richting = true;
  } else {                            // ..anders..
      pwmLa =-(aLa * Fclipped + bLa);
      pwmRa =-(aRa * Fclipped + bLa);
      pwmLv = 0;
      pwmRv = 0;
      richting = false;
  }
  analogWrite(motor1Lpen, pwmLv);
  analogWrite(motor2Lpen, pwmLa);
  analogWrite(motor1Rpen, pwmRv);
  analogWrite(motor2Rpen, pwmRa); 

  if (s < 1000)Serial.print(s / 100);
  Serial.print(" ");
  Serial.println(Fclipped);
  */

}
