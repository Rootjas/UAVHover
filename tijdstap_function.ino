double tijdstap(){

  dt = (current_time - millis())*0.001;
  current_time = millis();
  
  return dt;
}
