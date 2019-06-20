void loop() {
  
  //---------------------dt--------------------//
  double dt = tijdstap();
  
  //------------------Cel_monitor--------------//

  int spanning_status = cel_monitor(spanning_status);

  //------------------Stroom_meter-------------//

  int stroom_status = stroom_monitor(stroom_status);
/*
  if(spanning_status == 1){
    digitalWrite(7,LOW);
    Serial.println("");
    Serial.println("ERROR ERROR");
    Serial.println("");
  }
  else*/ digitalWrite(7,HIGH);
  
  //-----------------Serial-------------------//
  bool newData;
  char receivedChars[32], tempChars[32]; 
  float sp, Hoek_sp; 
  int event;
  recvWithStartEndMarkers(newData, receivedChars, tempChars);
  
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData(sp, Hoek_sp, event, tempChars);
    newData = false;
    Serial.print("\t\tPC: ");Serial.print(sp);Serial.print(",");Serial.print(Hoek_sp);Serial.print(",");Serial.print(event);
  }
 
  //------------------IMU----------------------//
  double Theta, Alpha, A_x, A_y, Omega, V_x, V_y, X_x, X_y;
  IMU_read(Theta, Omega, Alpha, A_x, A_y, V_x, V_y, X_x, X_y);
    
  //-----------------300mm regelaar-----------//
  //sp = 30;                        //VOOR TESTEN sp = 30;
  int PWMLv, PWMLa, PWMRv, PWMRa;
  //mm300_regelaar(dt, PWMLv, PWMLa, PWMRv, PWMRa, V_y, X_y, sp, 1);
  
  //-----------------hoek regelaar--------------//
  Hoek_sp = 0;                    //VOOR TESTEN Hoek_sp = 0;
  int R_PWMLv, R_PWMLa, R_PWMRv, R_PWMRa;
  Hoek_regelaar(dt, R_PWMLv, R_PWMLa, R_PWMRv, R_PWMRa, Theta, Hoek_sp);

  //----------------wand regelaar -------------//  
  int H_PWMLv, H_PWMLa, H_PWMRv, H_PWMRa;
//  Wand_volger(dt, H_PWMLv, H_PWMLa, H_PWMRv, H_PWMRa);
//  analogWrite(3,H_PWMLv);
//  analogWrite(9,H_PWMLa);
//  analogWrite(5,H_PWMRa);  
//  analogWrite(6,H_PWMRv);

  //------------------coordinaten regelaar---------------//
  event = 1;
  sp = 30;
  coordinaten_regelaar(dt, PWMLv, PWMLa, PWMRv, PWMRa, V_y, X_y, X_x, sp, Hoek_sp, event, Theta);


  //--------------------PWM voor 300 en hoek-------------//
  float weight = 0.3;
  int tot_PWMLv = Round(R_PWMLv * weight + PWMLv * (1.0 - weight));
  int tot_PWMLa = Round(R_PWMLa * weight + PWMLa * (1.0 - weight));
  int tot_PWMRa = Round(R_PWMRa * weight + PWMRa * (1.0 - weight));
  int tot_PWMRv = Round(R_PWMRv * weight + PWMRv * (1.0 - weight)); 
  analogWrite(3,tot_PWMLv);
  analogWrite(9,tot_PWMLa);
  analogWrite(5,tot_PWMRa);  
  analogWrite(6,tot_PWMRv);

  Serial.println("");
}
