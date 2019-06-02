#include "MPU9250.h"
#include "math.h"

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
  //Serial.print("Zr"); Serial.print(IMU.getGyroZ_rads(),5);
  Serial.print("\t\t");

  // Print mag values in degree/sec
  //Serial.print("X-m: "); Serial.print(IMU.getMagX_uT(),2);
  //Serial.print("\t\t");
  //Serial.print("Y-m: "); Serial.print(IMU.getMagY_uT(),2);
  //Serial.print("\t\t");
  //Serial.print("Z-m: "); Serial.print(IMU.getMagZ_uT(),2);
  //Serial.print("\t\t");

  Yaw = Yaw_Berekening(gZr, dt);
  //Serial.print("Yw"); Serial.print(Yaw);
  Serial.println("\t\t");
  Theta_versnelling = Alpha_Berekening(gZr, dt);

  Snelheid_X = Snelheid_Berekening(fXg, dt);
  Plaats_X = Plaats_Berekening(Snelheid_X, dt);
  Snelheid_Y = Snelheid_Berekening(fYg, dt);
  Plaats_Y = Plaats_Berekening(Snelheid_Y, dt);
  
}
