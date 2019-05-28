int spanning_teller = 0;

int cel_monitor(int spanning_status){
  //spanning_status : 0 = OK, 1 = NIET OK 

  float cel_waarde3 = analogRead(A1) * (5.0 / 1023.0);
  float cel_waarde2 = analogRead(A2) * (5.0 / 1023.0);
  float cel_waarde1 = analogRead(A3) * (5.0 / 1023.0);

  Serial.print("\t\t3cw");
  Serial.print(cel_waarde3, 2);
  Serial.print("\t\t2cw");
  Serial.print(cel_waarde2, 2);
  Serial.print("\t\t1cw");
  Serial.print(cel_waarde1, 2);

  
  if((cel_waarde1 < 3.3 || cel_waarde2 < 3.3 || cel_waarde3 < 3.3)&& spanning_status == 0){
    spanning_teller++;
  }
  else{
    spanning_teller = 0;
  }
  if(spanning_teller > 50)spanning_status = 1;
  

  return(spanning_status); 
}
