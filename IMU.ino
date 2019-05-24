#include "MPU9250.h"
#include "math.h"

MPU9250 IMU(Wire,0x68);
int status;
double Yaw, pitch, roll, Xg, Yg, Zg, dt, Zr, Zr_copy;
const float alpha = 0.5;
double current_time = millis();
double fXg = 0;
double fYg = 0;
double fZg = 0;
double gZr = 0;
float Zr_total = 0;
double Zr_count = 0;
double Theta, X_versnelling, Y_versnelling, Omega;

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

void IMU_read() {
  // read the sensor
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

  
  Serial.print("Xg"); Serial.print(Xg);
  //Serial.print(timeStep,3);
  Serial.print("\t\t");
  Serial.print("Yg"); Serial.print(Yg);
  Serial.print("\t\t");

  // Print gyro values in rad/sec
  Serial.print("Zr"); Serial.print(IMU.getGyroZ_rads(),5);
  Serial.print("\t\t");

  // Print mag values in degree/sec
  //Serial.print("X-m: "); Serial.print(IMU.getMagX_uT(),2);
  //Serial.print("\t\t");
  //Serial.print("Y-m: "); Serial.print(IMU.getMagY_uT(),2);
  //Serial.print("\t\t");
  //Serial.print("Z-m: "); Serial.print(IMU.getMagZ_uT(),2);
  //Serial.print("\t\t");

  Yaw = Yaw_Berekening(gZr, dt);
}
