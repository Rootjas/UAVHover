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

int x_as = 0;
const int x_max = 500;                         // aantal integratiestappen

//instellingen tbv serial read
int r = 1;


//--------------------------------------IMU SETTINGs--------------------------------------------------//
  float axb = 0.14; // accel bias
  float axs = 1; // accel scale factor
  float ayb = 0.28; // accel bias
  float ays = 1; // accel scale factor
//----------------------------------------------------------------------------------------------------//

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  // sensoren eiken (imu)
  //-----------------------------IMU----------------------//
  IMU_setup();
  // vanuit pi orientatie, positie en totale lengte (baan) tot volgende punt verkrijgen. (later nog een aantal keer uitvoeren zodat sensoren geeikt blijven)  HOEKEN in Radialen, AFSTANDEN in centimeters
}
