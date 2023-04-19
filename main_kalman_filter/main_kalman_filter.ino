#include <ESP32Encoder.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>
//#include <Servo.h>

#define M_PWM 26
#define M_SW 18
#define M_Dir 5
#define D1 33
#define D2 35

MPU6050 accelgyro;
ESP32Encoder encoder;

float count;
float theta_dotWheel=0;

const int m_freq = 20000;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;
const int M_Wheel = 1;

const int ledChannel_1 = 2;
const int ledChannel_2 = 3;

const int D_freq = 5000;

double Setpoint, Input, Output;
float kp=4;
float ki=0;
float kd=0.1;


unsigned long now, lastTime = 0;
float dt;                                   

int16_t ax, ay, az, gx, gy, gz;             //raw data from mpu6050
float aax=0, aay=0, agx=0, agy=0, agz=0;    //angle var
long axo = 0, ayo = 0, azo = 0;             //acc offset
long gxo = 0, gyo = 0, gzo = 0;             //gyr offset

float pi = 3.1415926;
float AcceRatio = 16384.0;                  //Ratio for acc 
float GyroRatio = 131.0;                    //Ratio for gyr 

uint8_t n_sample = 8;                       //num of filter sample 
float aaxs[8] = {0}, aays[8] = {0};         //x,y queoe
long aax_sum, aay_sum;                      //x,y sample sum

float a_x[10]={0}, a_y[10]={0} ,g_x[10]={0} ,g_y[10]={0}; //variance
float Px=1, Rx, Kx, Sx, Vx, Qx;             //Kalman variable for x
float Py=1, Ry, Ky, Sy, Vy, Qy;             //Kalman variable for y


PID M_PID(&Input, &Output, &Setpoint, kp , ki , kd , DIRECT);// kp ki kd

double offset=0;
double MPU_Input;

void setup() {
  Serial.begin(115200);
  /* M_Wheel setup */
  pinMode(M_SW, OUTPUT);
  pinMode(M_Dir, OUTPUT);
  ledcSetup(M_Wheel , m_freq, 9);
  ledcAttachPin(M_PWM, M_Wheel);

  ledcSetup(ledChannel_1, D_freq, resolution);
  ledcSetup(ledChannel_2, D_freq, resolution);
  ledcAttachPin(D1, ledChannel_1);
  ledcAttachPin(D2, ledChannel_2);


  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(19, 23); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

/*Initial MPU */
  Wire.begin();
  accelgyro.initialize();                

  unsigned short times = 200;             //sample times
  for(int i=0;i<times;i++){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //read raw data from mpu
    axo += ax; ayo += ay; azo += az;      
    gxo += gx; gyo += gy; gzo += gz;
  }
  axo /= times; ayo /= times; azo /= times; //calculate offset
  gxo /= times; gyo /= times; gzo /= times;
  

/*PID Controller*/
  Input=0;
  Setpoint=0.2;
  M_PID.SetMode(AUTOMATIC);
  M_PID.SetOutputLimits(-512,512);
  M_PID.SetSampleTime(10); // sample time for PID
}

void loop() {
/*Updating PID*/
  
  if (Serial.available()) {
    String reading = Serial.readStringUntil('\n');  // read from the Serial Monitor
    /* put your code here*/
    switch (reading[0]){
    case 'p':
      reading[0]=' ';
      reading.trim();
      kp=reading.toFloat();
      M_PID.SetTunings(kp, ki, kd);
      Serial.println("kp = " + String(kp) + ", ki = " + String(ki) + ", kd = " + String(kd));
      break;
    case 'i':
      reading[0]=' ';
      reading.trim();
      ki=reading.toFloat();
      M_PID.SetTunings(kp, ki, kd);
      Serial.println("kp = " + String(kp) + ", ki = " + String(ki) + ", kd = " + String(kd));
      break;
    case 'd':
      reading[0]=' ';
      reading.trim();
      kd=reading.toFloat();
      M_PID.SetTunings(kp, ki, kd);
      Serial.println("kp = " + String(kp) + ", ki = " + String(ki) + ", kd = " + String(kd));
      break;    
    default:
      Serial.println("kp = " + String(kp) + ", ki = " + String(ki) + ", kd = " + String(kd));
      break;
      }
  }



  unsigned long now = millis();             //Current time(ms)
  dt = (now - lastTime) / 1000.0;           //dt(s)
  lastTime = now;                           //last sampled time(ms)

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //raw data from MPU

  float accx = ax / AcceRatio;              
  float accy = ay / AcceRatio;              
  float accz = az / AcceRatio;             

  aax = atan(accy / accz) * (-180) / pi;    //angle between level and x
  aay = atan(accx / accz) * 180 / pi;       //angle between level and y

  aax_sum = 0;                              
  aay_sum = 0;
  for(int i=1;i<n_sample;i++){
    aaxs[i-1] = aaxs[i];
    aax_sum += aaxs[i] * i;
    aays[i-1] = aays[i];
    aay_sum += aays[i] * i;
  }
  aaxs[n_sample-1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; //0-90°
  aays[n_sample-1] = aay;                        
  aay_sum += aay * n_sample;                     
  aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;

  float gyrox = - (gx-gxo) / GyroRatio * dt; //x Angular velocity
  float gyroy = - (gy-gyo) / GyroRatio * dt; //y Angular velocity
  agx += gyrox;                             //Integration of x Angular velocity
  agy += gyroy;                             //Integration of x Angular velocity

  /* kalmen start */
  Sx = 0; Rx = 0;
  Sy = 0; Ry = 0;
  for(int i=1;i<10;i++){                 //avg sample 
    a_x[i-1] = a_x[i];                      
    Sx += a_x[i];
    a_y[i-1] = a_y[i];
    Sy += a_y[i];
  }
  a_x[9] = aax;
  Sx += aax;
  Sx /= 10;                                 //x avg spd
  a_y[9] = aay;
  Sy += aay;
  Sy /= 10;                                 //y avg spd

  for(int i=0;i<10;i++){
    Rx += sq(a_x[i] - Sx);
    Ry += sq(a_y[i] - Sy);
  }
  Rx = Rx / 9;                              //variance
  Ry = Ry / 9;                        

  Px = Px + 0.0025;                         
  Kx = Px / (Px + Rx);                      //calculate kalman gains
  agx = agx + Kx * (aax - agx);             
  Px = (1 - Kx) * Px;                       
  Py = Py + 0.0025;
  Ky = Py / (Py + Ry);
  agy = agy + Ky * (aay - agy);
  Py = (1 - Ky) * Py;

  /* kalmen end */





  MPU_Input=agx;
  /* Print out the values */
  Serial.print("Acceleration: ");
  Serial.print(MPU_Input);Serial.print(",");
  Serial.print(Output);
  Serial.print(",");
 
  
  theta_dotWheel = (encoder.getCount()-count) * 3.6 / dt;
  count=encoder.getCount();
  Serial.print(Output+theta_dotWheel*0.001);
   Serial.println();
  //M_Motor(-200);
  Self_Balancing(MPU_Input);
}

void M_Motor(double spd){
  spd=spd;
  if (abs(MPU_Input)>20)spd=0;
  if(spd ==0){digitalWrite(M_SW, LOW);ledcWrite(M_Wheel, 511);}
    else if(spd>0){
      digitalWrite(M_SW, HIGH);
      digitalWrite(M_Dir, HIGH);
      ledcWrite(M_Wheel, 512-spd);
    }
    else {
      digitalWrite(M_SW, HIGH);
      digitalWrite(M_Dir, LOW);
      ledcWrite(M_Wheel, 512+spd);
    }
}

void D_Motor(int spd){
  spd=-spd;
  if (spd > 0) {
    ledcWrite(ledChannel_1, LOW);
    ledcWrite(ledChannel_2, spd);
  }
  else {
    ledcWrite(ledChannel_2, LOW);
    ledcWrite(ledChannel_1, -spd);
  }
}
void turning(int degree){

}

void Self_Balancing(double input){
  Input = input;
  M_PID.Compute();
  float outputs=Output+theta_dotWheel*0.001;

  M_Motor(outputs);
}