
void loop() {

  //---------------------dt--------------------//
  double dt = tijdstap();
  digitalWrite(7,OUTPUT);

  // Hier komt een stukje code te staan die de stroomwaarden en spanningswaarden van de lipo accu uitleest en eventueel terugkoppelt aan de hmi?
  
  //------------------Cel_monitor--------------//

  int spanning_status = cel_monitor(spanning_status);

  //------------------Stroom_meter-------------//

  int stroom_status = stroom_monitor(stroom_status);

  if(spanning_status == 1 || stroom_status == 1){
    digitalWrite(7,LOW);
    Serial.println("ERROR ERROR")
  }
  else digitalWrite(7,HIGH);
  
  //------------------IMU----------------------//
  double Theta, Alpha, A_x, A_y, Omega, V_x, V_y, X_x, X_y;
  IMU_read(Theta, Omega, Alpha, A_x, A_y, V_x, V_y, X_x, X_y);


  int PWMLv, PWMLa, PWMRv, PWMRa;
  mm300_regelaar(dt, PWMLv, PWMLa, PWMRv, PWMRa);
  
  analogWrite(3,PWMLv);
  analogWrite(9,PWMLa);
  analogWrite(5,PWMRa);  
  analogWrite(6,PWMRv);
  /*
  // Informatie ophalen van pi, 1x keer in de zoveel seconden. Vanuit twee punten op het plafond kan een locatie(coordinaat) en een orientatie (de hoek tov de x-as) worden berekend(picam).
  if (Serial.available()){
    r = r * (Serial.read() - '0');
    Serial.println(r);
  }

  // Hier komt een stukje code van de afstandssensoren en hun regelaar(bij het detecteren van object binnen 0,3 cm ga de andere kant op accelereren/ kom tot stilstand)



  // regelsysteem input is de hoek of stand {radialen], dit vergelijk je met het setpoint gewenste hoek = 0[radialen]. Output is moment Mclipped
  herror_oud = herror;
  herror = hsp - theta;                           //setpoint sp word van te voren bepaald door picam dit is dus de totale lengte vd baan, de error is het verschil tussen sp en dead reckonde waarde theta 
  hd_error = herror - herror_oud;                 //de hd_error is het verschil tussen nieuw en oude error waarde die nodig is voor d -regelaar
  M = herror * hKp + hd_error / dt * hKd;         // de waarden voor hkp en hkd moeten we nog berekenen
  Mclipped = max(min(M, Mmax),-Mmax);             // -Mmax < M < +Mmax moment wordt begrenst om stabiliteit ten goede te komen +=rechtsom en -=linksom

  // hier moet een stukje code komen te staan die theta gelijk trekt met picam (kalibratie)
  // bijv: na 5 keer het bepalen van theta met behulp van het uitlezen van imu, bepalen we 1x theta met de picam.
  // De s die door de picam wordt gegeneerd is de kalibratie om ruisoptelling te voorkomen.

  // hier moet een stukje code komen te staan die het moment Mclipped omzet naar een pwm signaal naar motor 1 of 2 vooruit of achteruit


 

  // hier moet een stukje code komen te staan die s gelijk trekt met picam (kalibratie)
  // bijv: na 5 keer het bepalen van s met behulp van het uitlezen van imu, bepalen we 1x s met de picam.
  // De s die door de picam wordt gegeneerd is de kalibratie om ruisoptelling te voorkomen.

  // regelsysteem input is de al afgelegde weg s {CM], dit vergelijk je met het setpoint totaal af te leggen weg[CM]. Output is kracht Fclipped
  error_oud = error;
  error = sp - s;                             //setpoint sp word van te voren bepaald door picam dit is dus de totale lengte vd baan, de error is het verschil tussen sp en dead reckonde waarde s 
  d_error = error - error_oud;                //de d_error is het verschil tussen nieuw en oude error waarde die nodig is voor d -regelaar
  F = error * Kp + d_error / dt * Kd;         // de waarden voor kp en kd moeten we nog berekenen
  Fclipped = max(min(F, Fmax),-Fmax);         // -Fmax < F < +Fmax kracht wordt begrenst om stabiliteit ten goede te komen
  
  // omrekenen van kracht naar pwm signaal voor beide motoren, we gaan hiervan uit dat orientatie al voldoende is.


  //------------------USED FOR DEBUGGING-------------//
  printer();
  */
}
