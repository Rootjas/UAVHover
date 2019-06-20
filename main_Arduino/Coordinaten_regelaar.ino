
void coordinaten_regelaar(float dt, int &PWMLv, int &PWMLa, int &PWMRv, int &PWMRa, float V_y, float X_y, float X_x, float sp, float Hoek_sp, int event, double Theta){
  int R_PWMLv, R_PWMLa, R_PWMRv, R_PWMRa;

  float weight1 = 0;
  switch (event){
  case 1:
      Hoek_regelaar(dt, R_PWMLv, R_PWMLa, R_PWMRv, R_PWMRa, Theta, Hoek_sp);
      if( sqrt((Hoek_sp - Theta) * (Hoek_sp - Theta)) < 5 )mm300_regelaar(dt, PWMLv, PWMLa, PWMRv, PWMRa, V_y, X_y, sp, 0);


      int PWMLv = Round(R_PWMLv * weight1 + PWMLv * (1.0 - weight1));
      int PWMLa = Round(R_PWMLa * weight1 + PWMLa * (1.0 - weight1));
      int PWMRa = Round(R_PWMRa * weight1 + PWMRa * (1.0 - weight1));
      int PWMRv = Round(R_PWMRv * weight1 + PWMRv * (1.0 - weight1)); 
      break;
  /*case 2:
      Hoek_regelaar(dt, R_PWMLv, R_PWMLa, R_PWMRv, R_PWMRa, Theta, Hoek_sp);
      if( sqrt((Hoek_sp - Theta) * (Hoek_sp - Theta)) > 5 )afstand_regelaar(dt, PWMLv, PWMLa, PWMRv, PWMRa, V_y, sp);

      int PWMLv = Round(R_PWMLv * weight1 + PWMLv * (1.0 - weight1));
      int PWMLa = Round(R_PWMLa * weight1 + PWMLa * (1.0 - weight1));
      int PWMRa = Round(R_PWMRa * weight1 + PWMRa * (1.0 - weight1));
      int PWMRv = Round(R_PWMRv * weight1 + PWMRv * (1.0 - weight1)); 
      break;*/
  case 3:
      Wand_volger(dt, H_PWMLv, H_PWMLa, H_PWMRv, H_PWMRa);
      break;
  }
}
  
