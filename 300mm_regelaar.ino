const float Kp = 0.01, Ki = 0., Kd = 1.; // Regelaarparameters
const float sp = 30;       // setpoint = 30 cm

float error_oud, D_error;
float I_error = 0;
float error = 0;

int usTrigPen = 12;
int usEchoPen = 13;

/****************************************************************************/
float pwmLv, pwmLa,pwmRa,pwmRv;

bool richting;
    
const float aLv = 1909.2, bLv = -524.83, aRv = 2114.7,  bRv = -590.52; // vooruit bewegend
const float aLa = 751.31, bLa = -157.52, aRa = 737.27,  bRa = -153.19; // achteruit bewegend
/****************************************************************************/
// Houd rekening met de maximale stuwkracht voor zowel linker als rechter motor en met
// vooruit en achteruit.
/****************************************************************************/
const float FmaxL = 0.3690, FmaxR = 0.3647, FminL = -0.4590, FminR = -0.4635;
/****************************************************************************/


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

int limit_pwm(float pwm){

  if(pwm > 170) pwm = 170;
  if(pwm < 0) pwm = 0;
  return (Round(pwm));
}

int Ultrasone(int Trig_pin, int Echo_pin){
  
  pinMode(Trig_pin,OUTPUT);
  pinMode(Echo_pin, OUTPUT);
  digitalWrite(Trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_pin, LOW);
  float duration = pulseIn(Echo_pin,HIGH);
  return duration;
}

void mm300_regelaar(float dt, int &PWMLv, int &PWMLa, int &PWMRv, int &PWMRa){

  float s = Ultrasone(usTrigPen,usEchoPen);
  float C = 0.01
  
  //P-regelaar
  error = sp - s;
  
  //D-regelaar
  D_error = (error - error_oud) / dt;
  I_error = I_error + (error*dt);
  F = error * Kp + I_error * Ki + D_error * Kd;

  error_oud = error;
/*
  float Fv = C / error;
  if(error > 0 ){
    float Ftot = F - Fv;
    if(Ftot < 0) Ftot = -Ftot;
  }
*/
  Fclipped = max(min(F, Fmax), Fmin); // Fmin < F < +Fmax

  // Omrekening Fclipped naar pwm-signaal
  if (Fclipped > 0) {                 // Als de stuwkracht vooruit gericht is dan..
    //pwmLv = aLv * Fclipped + bLv;
    //pwmRv = aRv * Fclipped + bRv;
    pwmLa = (maxpwm / FmaxL) * Fclipped;
    pwmRa = (maxpwm / FmaxR) * Fclipped;
    pwmLv = 0;
    pwmRv = 0;
    richting = true;
  } else {                            // ..anders..
      //pwmLa = -(aLa * Fclipped + bLa);
      //pwmRa = -(aRa * Fclipped + bLa);
      pwmLv = (maxpwm / FminL) * Fclipped;
      pwmRv = (maxpwm / FminR) * Fclipped;
      pwmLa = 0;
      pwmRa = 0;
      richting = false;
  }
 
  PWMLv = limit_pwm(pwmLv);
  PWMRv = limit_pwm(pwmRv);
  PWMLa = limit_pwm(pwmLa);
  PWMRa = limit_pwm(pwmRa);
  
  Serial.print("\t\tPWMRv: ");
  Serial.print(pwmRv);
  Serial.print("\t\tPWMLv: ");
  Serial.print(pwmLv);
  Serial.print("\t\tPWMRa: ");
  Serial.print(pwmRa);
  Serial.print("\t\tPWMLa");
  Serial.print(pwmLa);
  Serial.print("\t\t");
}
