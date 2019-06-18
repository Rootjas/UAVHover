const float arm = 0.138; 

const float Hoek_Kp = 0.025, Hoek_Ki = 0., Hoek_Kd = 0.01; // Regelaarparameters
const float Theta_sp = 0;
float Hoek_error_oud;
float M, Mclipped;
float Hoek_Alpha;
const float Iz = 0.01955054;
const float massa = 544.5;

void Hoek_regelaar(float dt, int &PWMLv, int &PWMLa, int &PWMRv, int &PWMRa, double &Theta){

  //P-regelaar
  float Hoek_error = Theta_sp - Theta;

  //I-regelaar
  float Hoek_i_error = Hoek_error_oud + Hoek_error * dt;
  
  //D-regelaar
  float Hoek_d_error = (Hoek_error - Hoek_error_oud) / dt;
  
  float F_Hoek = (Hoek_error * Hoek_Kp + Hoek_i_error * Hoek_Ki + Hoek_d_error * Hoek_Kd);

  Hoek_error_oud = Hoek_error;

  Hoek_Alpha = M * Iz;
  
  float Fclipped_Hoek = max(min(F_Hoek, (Fmax)), (Fmin)); // Fmin < F < +Fmax & delen door 2 om beide motoren te laten werken.
  
  if (Fclipped_Hoek > 0) {      
    
    pwmLv = aLv * Fclipped_Hoek;
    pwmRa = aRv * Fclipped_Hoek;
    pwmLa = 0;
    pwmRv = 0;
    richting = true;

  } else {                            // ..anders..
      pwmLa = -(aLa * Fclipped_Hoek * Fclipped_Hoek + bLa * Fclipped_Hoek);
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

  Serial.print("\t\tRPWMLv:");Serial.print(PWMLv);
  Serial.print("\t\tRPWMRv:");Serial.print(PWMRv);
  Serial.print("\t\tRPWMLa:");Serial.print(PWMLa);
  Serial.print("\t\tRPWMRa:");Serial.print(PWMRa);

  Serial.print("\t\tF_R:");Serial.print(Fclipped_Hoek);
  
}
