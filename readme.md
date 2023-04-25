**Balance Tech**

The basic goal of this project was to create a two-wheel self balancing object reminiscent of a motorcycle with the ability to move and turn

<img src="/pics/cad.png"  width="300" height="200"><img src="/pics/bike1.png"  width="300" height="200"><img src="/pics/bike2.png"  width="300" height="200">


**Kalman Filter**

Accurate angle and angular velocity estimation.Precise sensor fusion of accelerometer and gyroscope data from the MPU6050 sensor.

*Basic structure: 

Data collection from MPU6050 sensor Calculate angles and angular velocities 

Integrate angular velocities to obtain angles

Calculate average and variance of accelerometer data

Update Kalman gains

Correct angles using Kalman gains and accelerometer data

Update covariance matrices

Filtered angle and angular velocity data used for LQR controller

**Dynamic system stability**

Implementing **Linear–quadratic regulator(LQR)** with adjustable gains for stable balancing under various conditions.
The LQR controller takes 4 parameters: Euler Angle, Angular velocity, momentum wheel actual speed and momentum wheel position, then calculates the PWM output. 

*Basic structure:

Calls Kalman_filter() function (process data)

Calculates the motor speed and position using the encoder data

Computes the control signal (pwm) based on the LQR control gains (K1-K4)

The momentum wheel motor is controlled by M_Motor() function





![img](https://i1.wp.com/www.esp32learning.com/wp-content/uploads/2018/12/MH-ET_LIVE_D1_mini_ESP32_pinout.png?resize=696%2C479)


**Pin Used**

M_Encoder GPIO 19, 23

​	PWM : GPIO 26

​	PWS  : GPIO 18

​	DIR    : GPIO 5

Drive Motor

​	GPIO 4, 33
Servo Motor
​	GPIO 17

<img src="/pics/circuit.png"  width="300" height="200">

### Installing Libraries

Open your Arduino IDE and go to **Sketch** > **Include Library** > **Manage Libraries**. The Library Manager should open. Search and install those libraries.

**mpu6050**

**ESP32Encoder**

**Bluepad32.h**

**Wire.h**


**I2Cdev.ht**

After installing the libraries, restart your Arduino IDE.	



