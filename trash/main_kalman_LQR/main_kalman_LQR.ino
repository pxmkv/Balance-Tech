#include <ESP32Encoder.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>


#define M_PWM 26
#define M_SW 18
#define M_Dir 5
#define D1 33
#define D2 35

MPU6050 accelgyro;
ESP32Encoder encoder;
float v_gyrox;
long enc_count;
float theta_dotWheel=0;

const int m_freq = 20000;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;
const int M_Wheel = 1;

const int ledChannel_1 = 2;
const int ledChannel_2 = 3;

const int D_freq = 5000;

int16_t motor_speed;
int32_t motor_pos;

int counter=0;

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

float MPU_Input;

/*LQR Const*/
float offset=0;

float K1 = 160;
float K2 = 25.00;
float K3 = 12.00;
float K4 = 0.60;
long currentT, previousT_1, previousT_2 = 0;  
float loop_time=10;
float gyroXfilt;

int pos = 107;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 17;

void setup() {
  Serial.begin(115200);
  /* M_Wheel setup */
  pinMode(M_SW, OUTPUT);
  pinMode(M_Dir, OUTPUT);
  ledcSetup(M_Wheel , m_freq, resolution);
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
  




}

void loop() {
  currentT = millis();
  Set_K_value();
  
  if (currentT - previousT_1 >= loop_time) {
  
  Kalman_filter();
  MPU_Input=agx+offset;


  enc_count=encoder.getCount();
  motor_speed=enc_count;
  encoder.setCount(0);

  gyroXfilt = 0.4 * v_gyrox + (1 - 0.4) * gyroXfilt;

  motor_pos += motor_speed;
  motor_pos = constrain(motor_pos, -110, 110);

  int pwm = constrain(K1 * MPU_Input + K2 * gyroXfilt + K3 * motor_speed + K4 * motor_pos, -255, 255); 
  M_Motor(-pwm);
  //aax, aay, agx, agy, agz
  /* Print out the values */
  if (counter>8){
  Serial.print("AGX: ");
  Serial.print(MPU_Input);Serial.print(",");
  Serial.print(v_gyrox);Serial.print(",");
  Serial.print(gyroXfilt);Serial.print(",");
  
  Serial.print(motor_speed);Serial.print(",");
  Serial.print(-pwm);//Serial.print(",");
  
  Serial.println();counter=0;
  }counter++;

  previousT_1 = currentT;
}}

void Kalman_filter(){
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
  v_gyrox=gyrox;
}

void D_Motor(int spd){
  spd=spd;
  if (spd > 0) {
    ledcWrite(ledChannel_1, LOW);
    ledcWrite(ledChannel_2, spd);
  }
  else {
    ledcWrite(ledChannel_2, LOW);
    ledcWrite(ledChannel_1, -spd);
  }
}


void M_Motor(double spd){
  spd=spd;
  if (abs(MPU_Input)>22)spd=0;
  if(spd ==0){digitalWrite(M_SW, LOW);ledcWrite(M_Wheel, 511);}
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

void turning(int degree){

}



void Set_K_value() {

  if (Serial.available()) {
    String reading = Serial.readStringUntil('\n');  // read from the Serial Monitor
    /* put your code here*/
    switch (reading[0]){
    case 'a':
      reading[0]=' ';
      reading.trim();
      K1=reading.toFloat();
      Serial.println("k1 = " + String(K1) + ", k2 = " + String(K2) + ", k3 = " + String(K3) + ", k4 = " + String(K4));
      break;
    case 's':
      reading[0]=' ';
      reading.trim();
      K2=reading.toFloat();
      
      Serial.println("k1 = " + String(K1) + ", k2 = " + String(K2) + ", k3 = " + String(K3) + ", k4 = " + String(K4));
      break;
    case 'd':
      reading[0]=' ';
      reading.trim();
      K3=reading.toFloat();
      
      Serial.println("k1 = " + String(K1) + ", k2 = " + String(K2) + ", k3 = " + String(K3) + ", k4 = " + String(K4));
      break;    
    case 'f':
      reading[0]=' ';
      reading.trim();
      K4=reading.toFloat();
      Serial.println("k1 = " + String(K1) + ", k2 = " + String(K2) + ", k3 = " + String(K3) + ", k4 = " + String(K4));
      break;  
    default:
      Serial.println("k1 = " + String(K1) + ", k2 = " + String(K2) + ", k3 = " + String(K3) + ", k4 = " + String(K4));
      break;
      }
  }

  
}