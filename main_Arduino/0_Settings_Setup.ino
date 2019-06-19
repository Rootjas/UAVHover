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
//--------------------------------------------------------------------------------------------------------//
//---------------------------------------Ultrasone---------------------------------------------------//
#define usAchterTrig 12
#define usAchterEcho 13
#define usMiddenTrig 10
#define usMiddenEcho 11
#define usVoorTrig 8
#define usVoorEcho 4


//----------------------------------------------------------------------------------------------------//
void setup() {
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(9,OUTPUT);

  pinMode(usAchterTrig,OUTPUT);
  pinMode(usAchterEcho,INPUT);
  pinMode(usMiddenTrig,OUTPUT);
  pinMode(usMiddenEcho,INPUT);
  pinMode(usVoorTrig,OUTPUT);
  pinMode(usVoorEcho,INPUT);

  Serial.begin(9600);

  
  //-----------------------------IMU----------------------//
  IMU_setup();
  // vanuit pi orientatie, positie en totale lengte (baan) tot volgende punt verkrijgen. (later nog een aantal keer uitvoeren zodat sensoren geeikt blijven)  HOEKEN in Radialen, AFSTANDEN in centimeters
}
