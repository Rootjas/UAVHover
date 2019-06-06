void snelheidsregelaar(float dt,float V_y, &PWMLv, &PWMLa, &PWMRv, &PWMRa){
  if(V_y > 1.1){
    PWMLv = 0.;
    PWMLa = 0.;
    PWMRv = 0.;
    PWMRa = 0.;
  }
}
