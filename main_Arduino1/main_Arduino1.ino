#include "MPU9250.h"
#include "math.h"
#include "Ultrasonic.h"

MPU9250 IMU(Wire,0x68);
int status;
double Yaw, pitch, roll; 
double Xg, Yg, Zg, dt, Zr;
const float alpha = 0.5;
double current_time = millis();
double fZg = 0;
double gZr = 0;
float Zr_total = 0;
double Zr_count = 0;
double Theta_versnelling, X_versnelling, Y_versnelling;
double fXg = 0;
double fYg = 0;
double Plaats = 0;
double Snelheid = 0;
const float arm = 0.138; 

// Instellingen

//Regelaarparameters roterend
const float hKp = 3., hKd = 5.;//Regelaarparameters
float ohmega = 0.0, theta = 0;
const float hsp = 0.00;


//Regelaarparameters 300 mm


/*
float error, error_oud, d_error, herror, herror_oud, hd_error;
const float m = 0.500;                         // kg
const float Fmax = 0.100;                      // N
float F, Fclipped, a, M, Mclipped, alfa;       // clippen = F begrenzen tussen -Fmax en +Fmax
*/
int x_as = 0;
const int x_max = 500;                         // aantal integratiestappen

//instellingen tbv serial read
int r = 1;


//--------------------------------------IMU SETTINGs--------------------------------------------------//
float axb = 1.40; // accel bias
float axs = 1; // accel scale factor
float ayb = 0.45; // accel bias
float ays = 1; // accel scale factor
//----------------------------------------------------------------------------------------------------//

void setup() {
  pinMode(12,OUTPUT);
  pinMode(13,INPUT);
  pinMode(3,OUTPUT);
  pinMode(9,OUTPUT);

  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  Serial.begin(9600);
  pinMode(7,OUTPUT);
  //-----------------------------IMU----------------------//
  IMU_setup();
}

void loop() {
  
  //---------------------dt--------------------//
  double dt = tijdstap();

  // Hier komt een stukje code te staan die de stroomwaarden en spanningswaarden van de lipo accu uitleest en eventueel terugkoppelt aan de hmi?
  
  //------------------Cel_monitor--------------//

  int spanning_status = cel_monitor(spanning_status);

  //------------------Stroom_meter-------------//

  int stroom_status = stroom_monitor(stroom_status);
/*
  if(spanning_status == 1 || stroom_status == 1){
    digitalWrite(7,LOW);
    Serial.println("");
    Serial.println("ERROR ERROR");
    Serial.println("");
  }
  else*/ digitalWrite(7,HIGH);
  
  //------------------IMU----------------------//
  double Theta, Alpha, A_x, A_y, Omega, V_x, V_y, X_x, X_y;
  IMU_read(Theta, Omega, Alpha, A_x, A_y, V_x, V_y, X_x, X_y);


  int PWMLv, PWMLa, PWMRv, PWMRa;
  //mm300_regelaar(dt, FPWMLv, FPWMLa, FPWMRv, FPWMRa);
  int RPWMLv, RPWMLa, RPWMRv, RPWMRa;
  Hoek_regelaar(dt, RPWMLv, RPWMLa, RPWMRv, RPWMRa, Theta);
  
//  PWMLv = (RPWMLv+PWMLv)/2;
//  PWMLa = (RPWMLa+PWMLa)/2;
//  PWMRa = (RPWMRa+PWMRa)/2;
//  PWMRv = (RPWMRv+PWMRv)/2; 
   
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
*/
  Serial.println("");
}

//----------------------------TIJDSTAP----------------------------//
long t_nw = millis();
long t_oud;
const long cyclustijd = 50; // ms

double tijdstap(){
  t_oud = t_nw;
  // Wacht tot de cyclustijd bereikt is:
  while (t_nw - t_oud < cyclustijd) t_nw = millis();
  dt = (t_nw - t_oud) * .001; // omzetten ms => s
  //Serial.print("\tDt");
  //Serial.print(dt);
  return dt;
}

//----------------------------300mm_regelaar----------------------//
#include "Ultrasonic.h"
const float Kp = 0.025, Ki = 0., Kd = 0.01; // Regelaarparameters
const float sp = 30;       // setpoint = 30 cm

float error_oud, D_error;
float I_error = 0;
float error = 0;

/****************************************************************************/
float pwmLv, pwmLa, pwmRa, pwmRv;

bool richting;


    
const float aLv = 993.44, aRv = 1007.9; // vooruit bewegend lineair
const float aLa = -2286.2, bLa = 1234.9, aRa = -1964.7,  bRa = 1169.1; // achteruit bewegend polynoom
/****************************************************************************/
// Houd rekening met de maximale stuwkracht voor zowel linker als rechter motor en met
// vooruit en achteruit.
/****************************************************************************/
const float FmaxL = 0.1722, FmaxR = 0.1678, FminL = -0.2278, FminR = -0.2295;
/****************************************************************************/


const float Fmax = min(FmaxL, FmaxR), Fmin = max(FminL, FminR);           // N
float F, Fclipped;

int maxpwm = 170;

const int motor1Lpen = 5;
const int motor1Rpen = 6;
const int motor2Lpen = 9;
const int motor2Rpen = 3;

int Round(float myfloat)
{
  double integral;
  float fraction = (float)modf(myfloat, &integral);
 
  if (fraction >= 0.5)
    integral += 1;
  if (fraction <= -0.5)
    integral -= 1;
 
  return (int)integral;
}

const int usTrigPen = 12;
const int usEchoPen = 13;
Ultrasonic ultrasonic(usTrigPen, usEchoPen);

void mm300_regelaar(float dt, int &PWMLv, int &PWMLa, int &PWMRv, int &PWMRa){
  
  float s = ultrasonic.Ranging(CM);
  if(s > 200){
    s = 200;
  }

  //P-regelaar
  error = (sp - s)*0.1;
  
  //D-regelaar
  D_error = (error - error_oud) / dt;

  //I-regelaar
  I_error = I_error + (error*dt);
  
  F = error * Kp + I_error * Ki + D_error * Kd;

  error_oud = error;
/*
  float Fv = C / error;
  if(error > 0 ){
    float Ftot = F - Fv;
    if(Ftot < 0) Ftot = -Ftot;
  }
*/
  Fclipped = max(min(F, Fmax), Fmin); // Fmin < F < +Fmax

  // Omrekening Fclipped naar pwm-signaal
  if (Fclipped > 0) {                 // Als de stuwkracht vooruit gericht is dan..
    pwmLv = aLv * Fclipped;
    pwmRv = aRv * Fclipped;
    //pwmLa = (maxpwm / FmaxL) * Fclipped;
    //pwmRa = (maxpwm / FmaxR) * Fclipped;
    pwmLa = 0;
    pwmRa = 0;
    richting = true;

  } else {                            // ..anders..
      pwmLa = -(aLa * Fclipped * Fclipped + bLa * Fclipped);
      pwmRa = -(aRa * Fclipped * Fclipped + bLa * Fclipped);
      //pwmLv = (maxpwm / FminL) * Fclipped;
      //pwmRv = (maxpwm / FminR) * Fclipped;
      pwmLv = 0;
      pwmRv = 0;
      richting = false;
  }
  PWMLv = max(min(Round(pwmLv), 170), 0);
  PWMRv = max(min(Round(pwmRv), 170), 0);
  PWMLa = max(min(Round(pwmLa), 170), 0);
  PWMRa = max(min(Round(pwmRa), 170), 0);
  /*Serial.print("\t\tPWMRv: ");
  Serial.print(pwmRv);
  Serial.print("\t\tPWMLv: ");
  Serial.print(pwmLv);
  Serial.print("\t\tPWMRa: ");
  Serial.print(pwmRa);
  Serial.print("\t\tPWMLa: ");
  Serial.print(pwmLa);*/
  Serial.print("\t\td:");Serial.print(s);
  Serial.print("\t\tF:");Serial.print(Fclipped);
}

//-------------------------------------------------------------------------//
//-------------------------------Cel_monitor---------------------------------------//
//-------------------------------------------------------------------------//
int spanning_teller = 0;

int cel_monitor(int spanning_status){
  //spanning_status : 0 = OK, 1 = NIET OK 

  float cel_waarde3 = analogRead(A1) * (5.0 / 1023.0);
  float cel_waarde2 = analogRead(A2) * (5.0 / 1023.0);
  float cel_waarde1 = analogRead(A3) * (5.0 / 1023.0);

  Serial.print("\t\t3cw");Serial.print(cel_waarde3, 2);
  Serial.print("\t\t2cw");Serial.print(cel_waarde2, 2);
  Serial.print("\t\t1cw");Serial.print(cel_waarde1, 2);

  
  if((cel_waarde1 < 3.3 || cel_waarde2 < 3.3 || cel_waarde3 < 3.3)&& spanning_status == 0){
    spanning_teller++;
  }
  else{
    spanning_teller = 0;
  }
  if(spanning_teller > 50)spanning_status = 1;
  

  return(spanning_status); 
}

//-------------------------------------------------------------------------//
//-------------------------------stroom_teller---------------------------------//
//-------------------------------------------------------------------------//

int stroom_teller;

int stroom_monitor(int stroom_status){
    //stroom_status : 0 = OK, 1 = NIET OK 

  float stroom_waarde = analogRead(A3) * (5.0 / 1023.0);

  Serial.print("\t\tsw"); Serial.print(stroom_waarde, 2);

  
  if(stroom_waarde > 0.7 && stroom_status == 0){
    stroom_teller++;
  }
  else{
    stroom_teller = 0;
  }
  if(stroom_teller > 50)stroom_status = 1;
  

  return(stroom_status); 
}



//-------------------------------------------------------------------------//
//-------------------------------IMU---------------------------------------//
//-------------------------------------------------------------------------//

void IMU_setup(){

  // start communication with IMU 
  status = IMU.begin();
  status = IMU.calibrateGyro();
  Serial.println(status);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  //Calibratie Versnelling X en Y
  IMU.setAccelCalX(axb,axs);
  IMU.setAccelCalY(ayb,ays);
}

double Yaw_Berekening(double Gyro_z, double dt){
  
  //Yaw is theta begin, theta wordt gebruikt als eindhoek
  Yaw = Yaw + ((Gyro_z) * dt * 180)/M_PI;
  return Yaw;
}

double Alpha_Berekening(double Gyro_z, double dt){

  double Alpha = ((Yaw + Gyro_z) / dt * 180)/M_PI;
  return Alpha;
}

double Snelheid_Berekening(double Versnelling, double dt){
  
  Snelheid = Snelheid + Versnelling * dt;
  return Snelheid;
}

double Plaats_Berekening(double Snelheid, double dt){
  
  Plaats = Plaats + Snelheid * dt;
  return Plaats;
  
}

void IMU_read(double &Yaw, double &gZr, double &Theta_versnelling, double &fXg, double &fYg, double &Snelheid_X, double &Snelheid_Y, double &Plaats_X, double &Plaats_Y) {

  IMU.readSensor();
  Xg = IMU.getAccelX_mss();
  Yg = IMU.getAccelY_mss();
  Zg = IMU.getAccelZ_mss();
  Zr = IMU.getGyroZ_rads();
  if(sqrt(Xg*Xg) < 0.05) Xg = 0;
  if(sqrt(Yg*Yg) < 0.05) Yg = 0;
  if(sqrt(Zr*Zr) < 0.05) Zr = 0;

  // display the data
  fXg = Xg * alpha + (fXg * (1.0 - alpha));
  fYg = Yg * alpha + (fYg * (1.0 - alpha));
  fZg = Zg * alpha + (fZg * (1.0 - alpha));
  gZr = Zr * alpha + (gZr * (1.0 - alpha));

  /*
  Serial.print("\t\tXg"); Serial.print(Xg);
  //Serial.print(timeStep,3);
  Serial.print("\t\t");
  Serial.print("Yg"); Serial.print(Yg);
  Serial.print("\t\t");
*/
  // Print gyro values in rad/sec
  //Serial.print("Zr"); Serial.print(IMU.getGyroZ_rads(),5);   Serial.print("\t\t");


  // Print mag values in degree/sec
  //Serial.print("X-m: "); Serial.print(IMU.getMagX_uT(),2);
  //Serial.print("\t\t");
  //Serial.print("Y-m: "); Serial.print(IMU.getMagY_uT(),2);
  //Serial.print("\t\t");
  //Serial.print("Z-m: "); Serial.print(IMU.getMagZ_uT(),2);
  //Serial.print("\t\t");

  Yaw = Yaw_Berekening(gZr, dt);
  Serial.print("\t\tYw"); Serial.print(Yaw);

  Theta_versnelling = Alpha_Berekening(gZr, dt);

  Snelheid_X = Snelheid_Berekening(fXg, dt);
  Plaats_X = Plaats_Berekening(Snelheid_X, dt);
  Snelheid_Y = Snelheid_Berekening(fYg, dt);
  Plaats_Y = Plaats_Berekening(Snelheid_Y, dt);
  
}

//-------------------------------------------------------------------------//
//-------------------------------HOEK---------------------------------------//
//-------------------------------------------------------------------------//
const float MmaxL = FmaxL * arm, MmaxR = FmaxR * arm, MminL = MminL * arm, MminR = FminR * arm;
const float Mmax = min(MmaxL, MmaxR), Mmin = max(MminL, MminR); 

const float Hoek_Kp = 0.25, Hoek_Ki = 0., Hoek_Kd = 0.1; // Regelaarparameters
const float Theta_sp = 0;       // setpoint = 30 cm
float Hoek_error_oud;
float M, Mclipped;

void Hoek_regelaar(float dt, int &PWMLv, int &PWMLa, int &PWMRv, int &PWMRa, double &Theta){

  //P-regelaar
  float Hoek_error = Theta_sp - Theta;

  //I-regelaar
  float Hoek_i_error = Hoek_error_oud + Hoek_error * dt;
  
  //D-regelaar
  float Hoek_d_error = (Hoek_error - Hoek_error_oud) / dt;
  
  float M = (Hoek_error * Hoek_Kp + Hoek_i_error * Hoek_Ki + Hoek_d_error * Hoek_Kd) * arm;

  Hoek_error_oud = Hoek_error;

  Hoek_Alpha = M * Iz;
  F = m * Hoek_Alpha;
  float Fclipped = max(min(F, (Fmax)), (Fmin)); // Fmin < F < +Fmax
  
  float Fclipped_Hoek = Mclipped / 2 / arm;   //verdeeld de moment over links en rechts
  
  if (Mclipped > 0) {      
    
    pwmLa = aLv * Fclipped_Hoek;
    pwmRv = aRv * Fclipped_Hoek;
    pwmLv = 0;
    pwmRa = 0;
    richting = true;

  } else {                            // ..anders..
      pwmLv = -(aLa * Fclipped_Hoek * Fclipped_Hoek + bLa * Fclipped_Hoek);
      pwmRa = -(aRa * Fclipped_Hoek * Fclipped_Hoek + bLa * Fclipped_Hoek);
      //pwmLv = (maxpwm / FminL) * Fclipped;
      //pwmRv = (maxpwm / FminR) * Fclipped;
      pwmLv = 0;
      pwmRv = 0;
      richting = false;
  }
  PWMLv = max(min(Round(pwmLv), 170), 0);
  PWMRv = max(min(Round(pwmRv), 170), 0);
  PWMLa = max(min(Round(pwmLa), 170), 0);
  PWMRa = max(min(Round(pwmRa), 170), 0);
  
  Serial.print("\t\tM: "); Serial.print(Mclipped,6);
  
}
