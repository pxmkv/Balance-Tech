#include <ESP32Encoder.h>
#define M_PWM 26
#define M_SW 18
#define M_Dir 5

ESP32Encoder encoder;

const int m_freq = 2000;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;
const int M_Wheel = 1;

void setup() {
  pinMode(M_SW, OUTPUT);
  pinMode(M_Dir, OUTPUT);

  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(19, 23); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching
  ledcSetup(M_Wheel , m_freq, resolution);
  ledcAttachPin(M_PWM, M_Wheel);

}

void loop() {
  
M_Motor(0);
}

void M_Motor(int spd){
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