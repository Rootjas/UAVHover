long t_nw = millis();
long t_oud;
const long cyclustijd = 50; // ms

double tijdstap(){
  t_oud = t_nw;
  // Wacht tot de cyclustijd bereikt is:
  while (t_nw - t_oud < cyclustijd) t_nw = millis();
  dt = (t_nw - t_oud) * .001; // omzetten ms => s
  Serial.print("\tDt");
  Serial.print(dt);
  return dt;
}
