// MPU6050_light - Version: Latest 

/*
  by: Manuel Rao
  stearted 3/10/2022
  Fanta it's a multipurpouse fligth softwere meant to be used with the following components:
  MPU6050 - accelerometer and gyroscope
  BMP180 - barometer for high altitude fligth
*/


#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <SD.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

//variables
//ofsets
float gyro_off[3] = {0, 0, 0};

//spacial variables
float IMU_world_acc[3] = {0, 0, 0};
float IMU_local_acc[3] = {0, 0, 0};
float IMU_gyro[3] = {0, 0, 0};
float IMU_world_rot[3] = {0, 0, 0};
float IMU_vel[3] = {0, 0, 0};
float IMU_pos[3] = {0, 0, 0};
float bar_pressure = 0;
float bar_altitude = 0;
float avrg_start_alt = 0;

//rotation matrix
float rotMatX[3] = {1, 0, 0};
float rotMatY[3] = {0, 1, 0};
float rotMatZ[3] = {0, 0, 1};

//extra variables
float step = 0;
#define pi 3.141592
#define rad2deg 57.2958

//servos
Servo serx1;
Servo sery1;
Servo serx2;
Servo sery2;

#define serx1_pin 9
#define serx2_pin 10
#define sery1_pin 11
#define sery2_pin 12

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens
  }

  sery1.attach(sery1_pin);
  sery2.attach(sery2_pin);
  serx1.attach(serx1_pin);
  serx2.attach(serx2_pin);

 if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  IMU_tune(1000, 2);
  Serial.println("Done!\n");
  Serial.println("program will start in 2sec...");
  delay(2000); 
}

void loop() { 
  unsigned long Past_time = micros();
  comonFilterUptate();
  unsigned long step_micros = micros() - Past_time;
  step = (float)step_micros / (float)1000000;
}

void IMU_tune(int fidelity, int sample_t) {
  Serial.println("");
  Serial.println("tuning IMU");
  Serial.println("do not move the IMU");

  for (int i = 0; i <= fidelity; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyro_off[0] = gyro_off[0] + g.gyro.x;
    gyro_off[1] = gyro_off[1] + g.gyro.y;
    gyro_off[2] = gyro_off[2] + g.gyro.z;
    delay(sample_t);
  }
  gyro_off[0] = -gyro_off[0] / fidelity;
  gyro_off[1] = -gyro_off[1] / fidelity;
  gyro_off[2] = -gyro_off[2] / fidelity;
  
  Serial.println("");
  Serial.println("IMU tuned");
  Serial.println("parameters:");
  Serial.print("X offset: ");
  Serial.print(gyro_off[0]);
  Serial.print("  Y offset: ");
  Serial.print(gyro_off[1]);
  Serial.print("  Z offset: ");
  Serial.print(gyro_off[2]);
  Serial.println("");
  }//old IMU tune

 void calculateRotMat(float x, float y, float z) {

  //create the variables that are re used in the math
  float cx = cos(x);
  float sx = sin(x);
  float cy = cos(y);
  float sy = sin(y);
  float cz = cos(z);
  float sz = sin(z);

  //linear algebra shit that ccalculates the actual matrix from the euler angles
  rotMatX[1] = cz * cy;
  rotMatX[2] = cz * sy * sx - sz * cx;
  rotMatX[3] = cz * sy * cx + sz * sx;
  rotMatY[1] = sz * cy;
  rotMatY[2] = sz * sy * sx + cz * cx;
  rotMatY[3] = sz * sy * cx + cz * sx;
  rotMatZ[1] = -sy;
  rotMatZ[2] = cy * sx;
  rotMatZ[3] = cy * cx;
  } //calculates a rotation matrix based on yaw, pitch and roll(XYZ) calculates rot mat
 //calculate rot mat

void get_IMU_data(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //local accelerations
  IMU_local_acc[0] = a.acceleration.x;
  IMU_local_acc[1] = a.acceleration.z;
  IMU_local_acc[2] = a.acceleration.y;

  //3x3 matrix multiplication to get world accelerations
  IMU_world_acc[0] = rotMatX[1] * IMU_local_acc[0] + rotMatX[2] * IMU_local_acc[1] + rotMatX[3] * IMU_local_acc[2] - 9.8;
  IMU_world_acc[1] = rotMatY[1] * IMU_local_acc[0] + rotMatY[2] * IMU_local_acc[1] + rotMatY[3] * IMU_local_acc[2];
  IMU_world_acc[2] = rotMatZ[1] * IMU_local_acc[0] + rotMatZ[2] * IMU_local_acc[1] + rotMatZ[3] * IMU_local_acc[2];

  //local(RAD)
  IMU_gyro[0] = (g.gyro.y + gyro_off[1]) * step;
  IMU_gyro[1] = (g.gyro.z + gyro_off[2]) * step;
  IMU_gyro[2] = (g.gyro.x + gyro_off[0]) * step;
  //3x3 matrix multiplication and integration to get world rotation
  IMU_world_rot[0] = IMU_gyro[0] + rotMatX[1] * IMU_gyro[0] + rotMatX[2] * IMU_gyro[1] + rotMatX[3] * IMU_gyro[2];
  IMU_world_rot[1] = IMU_gyro[1] + rotMatY[1] * IMU_gyro[0] + rotMatY[2] * IMU_gyro[1] + rotMatY[3] * IMU_gyro[2];
  IMU_world_rot[2] = IMU_gyro[2] + rotMatZ[1] * IMU_gyro[0] + rotMatZ[2] * IMU_gyro[1] + rotMatZ[3] * IMU_gyro[2];
}

void comonFilterUptate() { //this uptates the comon fliter, wich is echarged of estimating the position, velocity, rotation, etc. pf the vehicle.
  Serial.println("1");
    //get the IMU data
    get_IMU_data();
  Serial.println("1");
    //non linear ecuations
  IMU_vel[0] = IMU_vel[0] +  IMU_world_acc[0] * step;
  IMU_vel[1] = IMU_vel[1] +  IMU_world_acc[1] * step;
  IMU_vel[2] = IMU_vel[2] +  IMU_world_acc[2] * step;
  IMU_pos[0] = IMU_pos[0] + IMU_vel[0] * step;
  IMU_pos[1] = IMU_pos[1] + IMU_vel[1] * step;
  IMU_pos[2] = IMU_pos[2] + IMU_vel[2] * step;
  Serial.println("1");
  bar_pressure = bmp.readPressure();
  bar_altitude = bmp.readAltitude() - avrg_start_alt;
  Serial.println("1");
  //calculate the oriention matrix
  calculateRotMat(IMU_world_rot[0], IMU_world_rot[1], IMU_world_rot[2]);
  Serial.println("1");
  /* un-coment this for debuging*/
  //Serial.print(Xrot);
  //Serial.print(",");
  //Serial.print(Yrot);
  //Serial.print(",");
  //Serial.print("posZ:");
  Serial.print(IMU_world_rot[0] * 57.2958, 8);
  Serial.print("  ");
  Serial.print(IMU_gyro[0] * 57.2958, 8);
  Serial.print("  ");
  Serial.print(IMU_local_acc[0], 8);
  Serial.print("  ");
  Serial.print(IMU_world_acc[0], 8);
  Serial.print("  ");
  Serial.print(IMU_vel[0], 8);
  Serial.print("  ");
  Serial.println(IMU_pos[0], 8);
}


float PID(float p, float i, float d, float e /* error */, float le /* last error */, float ie /* error */) {

  float result = 0;
  result = p * e + i * ie * step + d * (e - le) / step;
  return result;
} //pid controler

/*
void goToAngle(float x, float y, float z) {
  //errors, integraed errors and past errors
  float ex = x - xrot;
  float lex = ex;
   float iex = iex + ex;
  float ey = y - yrot;
  float ley = ey;
  public float iey = iey + ey;
  float ez = z - zrot;
   lez = ez;
  public float iez = iez + ez;

  //PID controler
  float roll =  PID(1, 1, 1, ey, ley, iey);
  float yaw =   PID(1, 1, 1, ez, lez, iez);
  float pitch = PID(1, 1, 1, ex, lex, iex);

  serx1.write(constrain(yaw + pitch, -maxDeflection, maxDeflection));
  sery1.write(constrain(yaw + roll, -maxDeflection, maxDeflection));
  serx2.write(constrain(yaw + pitch, -maxDeflection, maxDeflection));
  sery2.write(constrain(yaw + roll, -maxDeflection, maxDeflection));
}

void CompileData(){
  String Data = "";
};
*/