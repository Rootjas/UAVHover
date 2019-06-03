int stroom_teller;

int stroom_monitor(int stroom_status){
    //stroom_status : 0 = OK, 1 = NIET OK 

  float stroom_waarde = analogRead(A3) * (5.0 / 1023.0);

  Serial.print("\t\tsw");
  Serial.print(stroom_waarde, 2);

  
  if((stroom_waarde > 0.7 && stroom_status == 0){
    stroom_teller++;
  }
  else{
    stroom_teller = 0;
  }
  if(stroom_teller > 50)stroom_status = 1;
  

  return(stroom_status); 
}
