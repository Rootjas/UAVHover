/*const float Wand_Kp = 0.025, Wand_Ki = 0., Wand_Kd = 0.02; // Regelaarparameters
const float Theta_Wand_sp = 0;
float Wand_error_oud;
float Theta_Wand_oud;

const float s_midden_achter = 0.0;

float Wand_transfer(float dt, int Theta){
  //P-regelaar
  float Wand_error = Theta_Wand_sp - Theta;

  //I-regelaar
  float Wand_i_error = Wand_error_oud + Wand_error * dt;
  
  //D-regelaar
  float Wand_d_error = (Wand_error - Wand_error_oud) / dt;
  
  float Theta_Wand = (Wand_error * Wand_Kp + Wand_i_error * Wand_Ki + Wand_d_error * Wand_Kd); // Eerste deel transfer function
  
  Wand_error_oud = Wand_error;
  
  Theta_Wand_oud = Theta;

  return Theta_Wand;
}

float us_Theta(){

  float s_achter = Ultrasone(usAchterEcho, usAchterTrig);
  float s_midden = Ultrasone(usMiddenEcho, usMiddenTrig);
  
  float s_us = s_midden - s_achter;
   
  double Theta =  atan2 (s_us, s_midden_achter);
  return Theta;
}

int Ultrasone(int Echo, int Trig){
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  float duration = pulseIn(Echo,HIGH);
  return (duration /29 / 2);
}

void Wand_regelaar(float dt, int &PWMLv, int &PWMLa, int &PWMRv, int &PWMRa){

  float Theta = us_Theta();

  float F_clipped_Hoek = Wand_transfer(dt, Theta);
  
  float Fclipped_Hoek = max(min(F_clipped_Hoek, Fmax), Fmin);
  
  if (Fclipped_Hoek > 0) {      
    
    pwmLv = aLv * Fclipped_Hoek-20;
    pwmRa = aRv * Fclipped_Hoek;
    pwmLa = 0;
    pwmRv = 0;
    richting = true;

  } else {                            // ..anders..
      pwmLa = -(aLa * Fclipped_Hoek * Fclipped_Hoek + bLa * Fclipped_Hoek)-20;
      pwmRv = -(aRa * Fclipped_Hoek * Fclipped_Hoek + bLa * Fclipped_Hoek);
      //pwmLv = (maxpwm / FminL) * Fclipped;
      //pwmRv = (maxpwm / FminR) * Fclipped;
      pwmLv = 0;
      pwmRa = 0;
      richting = false;
  }
  
  PWMLv = max(min(Round(pwmLv), 170), 0);
  PWMRv = max(min(Round(pwmRv), 170), 0);
  PWMLa = max(min(Round(pwmLa), 170), 0);
  PWMRa = max(min(Round(pwmRa), 170), 0);

//  Serial.print("\t\tRPWMLv:");Serial.print(PWMLv);
//  Serial.print("\t\tRPWMRv:");Serial.print(PWMRv);
//  Serial.print("\t\tRPWMLa:");Serial.print(PWMLa);
//  Serial.print("\t\tRPWMRa:");Serial.print(PWMRa);
//
//  Serial.print("\t\tF_R:");Serial.print(Fclipped_Hoek);
  
}*/
