#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>

#define M_PWM 26
#define M_SW 18
#define M_Dir 5
#define D1 33
#define D2 35


ESP32Encoder encoder;
Adafruit_MPU6050 mpu;

const int m_freq = 20000;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;
const int M_Wheel = 1;
const int D_1 = 2;
const int D_2 = 3;
const int D_freq = 2000;

double Setpoint, Input, Output;
PID M_PID(&Input, &Output, &Setpoint, 10 , 0.4 , 0.1 , DIRECT);// kp ki kd

double offset=0;
double MPU_Input;

void setup() {
  Serial.begin(115200);
  /* M_Wheel setup */
  pinMode(M_SW, OUTPUT);
  pinMode(M_Dir, OUTPUT);
  ledcSetup(M_Wheel , m_freq, resolution);
  ledcAttachPin(M_PWM, M_Wheel);

  ledcSetup(D_1, D_freq, resolution);
  ledcSetup(D_2, D_freq, resolution);
  ledcAttachPin(D1, D_1);
  ledcAttachPin(D2, D_2);
  
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(19, 23); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching


  // Try to initialize the MPU!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {delay(10);}}
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  Serial.println("");
  delay(100);


/*PID Controller*/
  Input=0;
  Setpoint=0;
  M_PID.SetMode(AUTOMATIC);
  M_PID.SetOutputLimits(-255,255);
  M_PID.SetSampleTime(10);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  offset=a.acceleration.y;
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  MPU_Input=a.acceleration.y-offset;
  /* Print out the values */
  Serial.print("Acceleration: ");
  Serial.println(MPU_Input);
  Serial.print("");
  //M_Motor(a.acceleration.x*20);

  Self_Balancing(MPU_Input);
}

void M_Motor(double spd){
  if(spd ==0){digitalWrite(M_SW, LOW);}
    else if(spd>0){
      digitalWrite(M_SW, HIGH);
      digitalWrite(M_Dir, HIGH);
      ledcWrite(M_Wheel, 255-spd);
    }
    else {
      digitalWrite(M_SW, HIGH);
      digitalWrite(M_Dir, LOW);
      ledcWrite(M_Wheel, 255+spd);
    }
}

void D_Motor(int speed){
  if (speed > 0) {
    ledcWrite(D_1, LOW);
    ledcWrite(D_2, speed);
  }
  else if (speed == 0){
    ledcWrite(D_1, LOW);
    ledcWrite(D_2, LOW);
  }
  else {
    ledcWrite(D_2, LOW);
    ledcWrite(D_1, speed);
  }

}
void turning(int degree){

}

void Self_Balancing(double input){
  Input = input;
  M_PID.Compute();
  M_Motor(Output);
}