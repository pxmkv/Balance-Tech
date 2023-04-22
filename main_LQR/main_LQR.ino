#include <ESP32Encoder.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Bluepad32.h>

#define ledPin 2
#define M_PWM 26
#define M_SW 18
#define M_Dir 5
#define D1 33
#define D2 4

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

MPU6050 accelgyro;
ESP32Encoder encoder;
float v_gyrox;
long enc_count;

const int m_freq = 20000;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;
const int M_Wheel = 1;
const int DChannel_1 = 2;
const int DChannel_2 = 3;
const int servoChannel = 4;
const int servo_freq = 50;
const int D_freq = 5000;

int16_t motor_speed;
int32_t motor_pos;

                             

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
float K2 = 8.00;
float K3 = 1.00;
float K4 = 0.60;
long currentT, previousT_1, previousT_2 = 0;  
float loop_time=10;
float gyroXfilt;



int servoPin = 17;


int counter=0;  

void setup() {
  Serial.begin(115200);

  /* M_Wheel setup */
  pinMode(ledPin, OUTPUT);
  pinMode(M_SW, OUTPUT);
  pinMode(M_Dir, OUTPUT);
  ledcSetup(M_Wheel , m_freq, resolution);
  ledcAttachPin(M_PWM, M_Wheel);

  ledcSetup(DChannel_1, D_freq, resolution);
  ledcSetup(DChannel_2, D_freq, resolution);
  ledcAttachPin(D1, DChannel_1);
  ledcAttachPin(D2, DChannel_2);

  ledcSetup(servoChannel, servo_freq, resolution);
  ledcAttachPin(servoPin, servoChannel);
  

  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(19, 23); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

/*Initial MPU */
  Wire.begin();
  accelgyro.initialize();   
               
/*Initial Kalman Filter*/
  unsigned short times = 200;             //sample times
  for(int i=0;i<times;i++){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //read raw data from mpu
    axo += ax; ayo += ay; azo += az;      
    gxo += gx; gyo += gy; gzo += gz;
  }
  axo /= times; ayo /= times; azo /= times; //calculate offset
  gxo /= times; gyo /= times; gzo /= times;
  



  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();
  xTaskCreatePinnedToCore(remote, "remote function", 10000, NULL, 1, NULL,  0); // CPU Core 0
}


void loop() {//CPU Core 1
 
  currentT = millis();
  Set_K_value(); // a=k1 s=k2 d=k3 f=k4 o=offset c=calibrate


  if (currentT - previousT_1 >= loop_time) {
  Kalman_filter();
  MPU_Input=-agx+offset;
  enc_count=encoder.getCount();
  motor_speed=enc_count;
  encoder.setCount(0);
  gyroXfilt = 0.4 * v_gyrox + (1 - 0.4) * gyroXfilt;
  motor_pos += motor_speed;
  motor_pos = constrain(motor_pos, -110, 110);

  int pwm = constrain(K1 * MPU_Input + K2 * gyroXfilt + K3 * motor_speed + K4 * motor_pos, -255, 255); //Linear–quadratic regulator
  M_Motor(-pwm);
  
  //aax, aay, agx, agy, agz
  /* Print out the values */
  
  if (counter>5){
  Serial.print("AGX: ");
  Serial.print(MPU_Input);Serial.print(",");
  Serial.print(gyroXfilt);Serial.print(",");
  Serial.print(motor_pos);Serial.print(",");
  Serial.print(motor_speed);Serial.print(",");
  Serial.print(-pwm);//Serial.print(",");
  Serial.println();counter=0;
  }counter++;

  previousT_1 = currentT;
  
  }
}

void Kalman_filter(){

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

  float gyrox = - (gx-gxo) / GyroRatio * loop_time/1000; //x Angular velocity
  float gyroy = - (gy-gyo) / GyroRatio * loop_time/1000; //y Angular velocity
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
  v_gyrox=-gyrox*1000/loop_time;
}

void D_Motor(int spd){
  spd=spd;
  if (spd > 0) {
    ledcWrite(DChannel_1, LOW);
    ledcWrite(DChannel_2, spd);
  }
  else {
    ledcWrite(DChannel_2, LOW);
    ledcWrite(DChannel_1, -spd);
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


void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}


void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        Serial.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

void turn(int position){ledcWrite(servoChannel, map(position, 0, 180, 5, 32));}

void remote(void * parameter){
  while(1){
    BP32.update();
    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];

        if (myGamepad && myGamepad->isConnected()) {
            digitalWrite(ledPin,HIGH);
            // There are different ways to query whether a button is pressed.
            // By query each button individually:
            //  a(), b(), x(), y(), l1(), etc...

            // Another way to query the buttons, is by calling buttons(), or
            // miscButtons() which return a bitmask.
            // Some gamepads also have DPAD, axis and more.
            if (myGamepad->throttle()){D_Motor((myGamepad->throttle())/4);}
            else if (myGamepad->brake()){D_Motor(-(myGamepad->brake())/4);}
            else{D_Motor(0);}
            turn(map(myGamepad->axisRX(),-511,511,45,135));
            /*
            Serial.printf(
                "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
                "%4d, brake: %4d, throttle: %4d, misc: 0x%02x\n",
                i,                        // Gamepad Index
                myGamepad->dpad(),        // DPAD
                myGamepad->buttons(),     // bitmask of pressed buttons
                myGamepad->axisX(),       // (-511 - 512) left X Axis
                myGamepad->axisY(),       // (-511 - 512) left Y axis
                myGamepad->axisRX(),      // (-511 - 512) right X axis
                myGamepad->axisRY(),      // (-511 - 512) right Y axis
                myGamepad->brake(),       // (0 - 1023): brake button
                myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
                myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
            );
            */
            // You can query the axis and other properties as well. See Gamepad.h
            // For all the available functions.
        }
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    // vTaskDelay(1);
    delay(150);}
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
    case 'o':
      reading[0]=' ';
      reading.trim();
      offset=reading.toFloat();
      Serial.println("offset = " + String(offset));
      break;
    case 'c': //calibrate
      reading[0]=' ';
      reading.trim();
      offset=agx;
      Serial.println("offset = " + String(offset));
      break;         
    default:
      Serial.println("k1 = " + String(K1) + ", k2 = " + String(K2) + ", k3 = " + String(K3) + ", k4 = " + String(K4));
      break;
      }
  }
}