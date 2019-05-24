
// Instellingen

//Regelaarparameters roterend
const float hKp = 3., hKd = 5.;//Regelaarparameters
float ohmega = 0.0, theta = 0;
const float hsp = 0.00;


//Regelaarparameters lineair
const float Kp = 4., Kd = 5.;                  // Regelaarparameters
float v = 0.0, s = 2.0;                        // Beginwaarden
const float sp = 0.300;                        // setpoint = 30 cm

float error, error_oud, d_error, herror, herror_oud, hd_error;
const float m = 0.500;                         // kg
const float Fmax = 0.100;                      // N
float F, Fclipped, a, M, Mclipped, alfa;       // clippen = F begrenzen tussen -Fmax en +Fmax

long t_nw = millis();
long t_oud;
float dt;
const long cyclustijd = 50;                    // ms

int x_as = 0;
const int x_max = 500;                         // aantal integratiestappen

//instellingen tbv serial read
int r =1;





void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  // sensoren eiken (imu)
  // imu opstart kalibratie, geen verandering in locatie van picam betekend, versnelling ay van imu moet 0 zijn.
  // vanuit pi orientatie, positie en totale lengte (baan) tot volgende punt verkrijgen. (later nog een aantal keer uitvoeren zodat sensoren geeikt blijven)  HOEKEN in Radialen, AFSTANDEN in centimeters
}

void loop() {
  // put your main code here, to run repeatedly:
  t_oud = t_nw;
  // Wacht tot de cyclustijd bereikt is:
  while (t_nw - t_oud < cyclustijd) t_nw = millis();
  dt = (t_nw - t_oud) * .001;                       //omzetten van ms => sec

  

  // Hier komt een stukje code te staan die de stroomwaarden en spanningswaarden van de lipo accu uitleest en eventueel terugkoppelt aan de hmi?


  
  // Informatie ophalen van pi, 1x keer in de zoveel seconden. Vanuit twee punten op het plafond kan een locatie(coordinaat) en een orientatie (de hoek tov de x-as) worden berekend(picam).
  if (Serial.available()){
    r = r * (Serial.read() - '0');
    Serial.println(r);
  }

  // Hier komt een stukje code van de afstandssensoren en hun regelaar(bij het detecteren van object binnen 0,3 cm ga de andere kant op accelereren/ kom tot stilstand)






  // dead reckoning berekening om tussendoor de stand of hoek te bepalen
  alfa =                        // lees de hoekversnellingssensor (IMU) om de z -as uit: i2c
  ohmega = ohmega + alfa * dt   // Hoeknselheid = oude hoeksnelheid + snelheidsverschil ten gevolgen van hoekversnelling
  theta = theta + ohmega * dt   // Stand of hoek = oude hoek + hoek ten gevolge van hoeksnelheid

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


 
  // dead reckoning berekening om tussendoor positie te bepalen tot het eindpunt is verkregen.
  a =                           // lees de versnellingssensor (IMU) van de y-as uit: i2c
  v = v + a * dt;               // nieuwe snelheid = oude snelheid + snelheidsverschil ten gevolgen van versnelling of vertraging
  s = s + v * dt;               // nieuwe afgelegde weg = oude afgelegde weg + afgelegde weg (snelheid aangehouden binnen bepaald tijdsbestek)

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
}
