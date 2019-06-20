//const float aLv = 993.44, aRv = 1007.9; // vooruit bewegend
//const float aLa = -2286.2, bLa = 1234.9, aRa = -1964.7,  bRa = 1169.1; // achteruit bewegend
//const float FmaxL = 0.3690, FmaxR = 0.3647, FminL = -0.4590, FminR = -0.4635;
//const float Fmax = min(FmaxL, FmaxR), Fmin = max(FminL, FminR);           // N
double F_wand, Fclipped_wand;            // clippen = F begrenzen tussen -Fmax en +Fmax
const double Wand_Kp = 0.01, Wand_Ki = 0.0004095, Wand_Kd = 0.04588;
double derivative, integral, error_prior, H;
int desired_value = 0;
float tijd, iteration_time;
float afstand_Us1_Us2 = 0.204;
float I = 0.019551;
float metersRA, metersRV;
float len_motorR = 0.1399183, len_motorL = 0.1280817;

void read_us(){
  digitalWrite(usAchterTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(usAchterTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(usAchterTrig, LOW);

  tijd = pulseIn(usAchterEcho, HIGH);
  metersRA = (tijd/29/2)/100;
  metersRA = round(metersRA*100.0)/100.0;
  Serial.print("\t\tus_ac: ");Serial.print(metersRA);

  digitalWrite(usMiddenTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(usMiddenTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(usMiddenTrig, LOW);

  tijd = pulseIn(usMiddenEcho, HIGH);
  metersRV = (tijd/29/2)/100;
  metersRV = round(metersRV*100.0)/100.0;
  Serial.print("\t\tus_mid: ");Serial.print(metersRV);  
}

double PIDregelaar(float &Wand_error){
  integral = integral + (Wand_error*iteration_time);
  derivative = (Wand_error - error_prior)/iteration_time;
  //Serial.print(" derivative: ");Serial.print(derivative);
  double output = Wand_Kp*error + Wand_Ki*integral + Wand_Kd*derivative;
  error_prior = Wand_error;
  return output;
}

int limit_pwm(float pwm){
  if(pwm > 170) pwm = 170;
  if(pwm < 0) pwm = 0;
  return (Round(pwm));
}


void Wand_volger(float iteration_time, int &pwmLv, int &PWMLa, int &pwmRv, int &PWMRa){
  
  read_us();
  
  if (metersRA < 0.25 && metersRV < 0.25){
    pwmLv = 30;
    pwmRv = 50;
    Serial.println("Rechtsachter < 0.20");
  }  
  /*else if (metersRV < 0.20 && metersRA > 0.20){
    pwmLv = 20;
    pwmRv = 70;
    analogWrite(RFPWM_Output, pwmRv);
    analogWrite(LFPWM_Output, pwmLv);
    Serial.println("Rechtsvoor < 0.20");
    previous = current;
  }*/
  else if (metersRV < 0.25 || metersRA < 0.25){
    pwmLv = 20;
    pwmRv = 40;
  }
  else if(metersRA > 0.75 || metersRV > 0.75){
    pwmLv = 40;
    pwmRv = 20;
  }  
  else if (metersRV > 0.75 && metersRA > 0.75){
    pwmLv = 50;
    pwmRv = 30;
  }
  else{
    float Wand_error = desired_value - (metersRV - metersRA);
    Wand_error = round(Wand_error*100.0)/100.0;
    Serial.print("\t\tWand_error: ");Serial.print(error);
    double output = PIDregelaar(Wand_error);
    H = (afstand_Us1_Us2/I)*(iteration_time);
    M = output * H;
    Serial.print("\t\tMoment: ");Serial.print(M,4);
    if (M<0){
      F_wand = M/len_motorL;
      F_wand = -F_wand;
      Fclipped = max(min(F, Fmax), Fmin);
      Serial.print(" FL: ");
      Serial.println(Fclipped);
      pwmLv = 45 + (aLv * Fclipped);
      pwmLv = limit_pwm(pwmLv);
      pwmRv = 0;
    }
    else if (M > 0){
      F_wand = M / len_motorR;
      Fclipped_wand = max(min(F_wand, Fmax), Fmin);
      Serial.print("\t\tFR: ");Serial.print(Fclipped_wand);
      pwmRv = 45 + (aRv * Fclipped_wand);
      pwmRv = limit_pwm(pwmRv);
      pwmLv = 0;
    }
  }
  Serial.print("\t\tpwmLv: ");Serial.print(pwmLv);
  Serial.print("\t\tpwmRv: ");Serial.println(pwmRv);  
}
