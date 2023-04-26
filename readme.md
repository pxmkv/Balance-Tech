**Balance Tech**

The basic goal of this project was to create a two-wheel self balancing object reminiscent of a motorcycle with the ability to move and turn

<img src="/pics/cad.png"  width="300" height="200"><img src="/pics/bike1.png"  width="300" height="200"><img src="/pics/bike2.png"  width="300" height="200"><img src="/pics/bike3.png"  width="300" height="200">


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

**CPU1:** 

  Calls Kalman_filter() function (process data)

  Calculates the motor speed and position using the encoder data

  Computes the control signal (pwm) based on the LQR control gains (K1-K4)

  About LQR calculation [/main_LQR/readme.md](https://github.com/pxmkv/Balance-Tech/tree/main/main_LQR)

  The momentum wheel motor is controlled by M_Motor() function

**CPU0:**

  Read Game Controller input using Bluepad32 library
  mapping joystick reading value to the drivetrain motor and servo motor


<img src="https://i1.wp.com/www.esp32learning.com/wp-content/uploads/2018/12/MH-ET_LIVE_D1_mini_ESP32_pinout.png"  width="600" height="400">



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


something useful:

  https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/LQR.pdf
 
  https://authors.library.caltech.edu/5154/1/OLFcdc01b.pdf
 
  https://www.researchgate.net/publication/351314795_Multibody_Modeling_and_Balance_Control_of_a_Reaction_Wheel_Inverted_Pendulum_Using_LQR_Controller
  

