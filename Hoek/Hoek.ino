/*
    Simulatie van plaats s en snelheid v van een massa m en een stuwkracht F
    Bewegingsvergelijking:
       m.a = F
       
    Integratie obv 
       a = dv/dt
       v = ds/dt
       
    => dv = a.dt => v_nw - v_oud = a.dt => v_nw = v_oud + a.dt =>
       v = v + a*dt
       s = s + v*dt
*/

const float Iz = 0.500;      // Traagheidsmoment, moet veranderd worden
/*const*/ float M = 0.1;        // Moment
float alpha;
float w = 0.0, theta = 0.0;     // Beginwaarden

long t_nw = millis();
long t_oud;
float dt;
const long cyclustijd = 50; // ms

int x_as = 0;
const int x_max = 2000;      // aantal integratiestappen

float F, error, error_oud = 1.0, d_error;
const float Kp = 0.5;
const float Kd = 2.0;
const float thetap = 90.0;

void setup() {
  Serial.begin(57600);
}

void loop() {
  t_oud = t_nw;
  // Wacht tot de cyclustijd bereikt is:
  while (t_nw - t_oud < cyclustijd) t_nw = millis();
  dt = (t_nw - t_oud) * .001;

  //P-regelaar
  error = thetap - theta;
  
  //D-regelaar
  d_error = (error - error_oud) / dt;
  
  M = error * Kp + d_error * Kd;

  error_oud = error;

  
  alpha = M / Iz;      // of lees de versnellingssensor uit: a = analogRead(a_pen);
  w = w + alpha * dt; // deze twee vgln eigenlijk verwisselen van plaats (evt: v += a * dt)
  theta = theta + w * dt; // deze twee vgln eigenlijk verwisselen van plaats (evt: s += v * dt)

  Serial.print("stuwkracht: ");
  //Serial.print(M,2);
  Serial.print("\t");
  /*Serial.print("Acceleratie: ");
  Serial.print(a,2);
  Serial.print("\t");
  Serial.print("Snelheid: ");
  Serial.print(v,2);
  Serial.print("\t");*/
  Serial.print("afstand: ");
  if(theta < 200)Serial.println(theta,2);

  // Stop met plotten, anders loopt de grafiek van het scherm af
  x_as++;
  while (x_as > x_max);
}
