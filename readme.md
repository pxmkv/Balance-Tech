<img src="/pics/logo.png">

**Balance-Tech**

The basic goal of this project was to create a two-wheel self balancing object reminiscent of a motorcycle with the ability to move and turn

<img src="/pics/cad.png"  width="300" height="200"><img src="/pics/bike1.png"  width="300" height="200">

<img src="/pics/bike2.png"  width="300" height="200"><img src="/pics/bike3.png"  width="300" height="200">

Integrated device with labeled actuators and
sensors:

<img src="/pics/bike.png"  width="600" height="400">

The bulk of the circuit design lies within a three “story” structure. Three circuit boards with a gap of about an inch between each house all the circuitry and key parts within. Battery power is wired in a position that will decrease center of mass and aid in smoother balancing. The Buck converter allows for an adjustable power supply and the AMS1117 serves as a voltage regulator. Other smaller parts exist in the circuit but the critical parts are provided here

<img src="/pics/circuit.png"  width="300" height="200"><img src="/pics/diagram.png"  width="300" height="200">

** Engineering Analysis **
A. External Torque Requirements

<img src="/pics/diagram2.png"  width="600" height="660">

The system’s center of mass is above its pivot point, which makes it inherently unstable; any tilt will result in a gravitational torque. The Euler angle quantifies the angular displacement of the system about the axis of rotation. If the Euler angle reaches a threshold of 22°, the motor is programmed to stop. Taking that as the maximum angle of operation, the maximum gravitational torque is as follows:

<img src="/pics/equation1.png"  width="600" height="60">

The following calculations verify the reaction wheel’s ability to apply a neutralizing torque of equal magnitude or higher

<img src="/pics/equation2.png"  width="600" height="500">

Additionally, the performance curve of the brushless DC motor shows that it is capable of delivering enough torque to the reaction wheel.

<img src="/pics/curve.png"  width="600" height="600">

B. Optimization of the K Gain Matrix

There are four parameters that provide information about the system's state and help to determine the appropriate control actions to maintain balance: the Euler angle, the angular velocity of the system, the motor speed, and the motor position. The Euler angle and angular velocity describe the effect of the gravitational torque; the desired states are 0° and 0 rad/s respectively. The motor speed and motor position describe the effect of the torque
applied by the momentum wheel; the desired states of these parameters depend on the system’s dynamics. In order to maintain balance of the system, a feedback control approach is used to bring the actual states closer to the desired states. 
The K gain matrix is the set of feedback gains that are used to adjust the speed and direction of the momentum wheel based on the difference between the desired and actual states of the system. The LQR algorithm is used to find the optimal K gain matrix that minimizes the control effort (i.e. motor speed/torque) while also minimizing the system’s deviation from the desired state. The calculation of the optimal K gain matrix was done using the lqr() function in MATLAB. This function requires four inputs: two linearized state-space matrices A and B, the state-cost weighted matrix Q, and the control weighted matrix R. 
The K feedback gain is multiplied by the difference between the actual state and desired state to generate the control signal (u) that will minimize the quadratic cost function J(u) [3], which is constricted to linear system dynamics [13]. The optimal K gain matrix is found using the lqr() function in MATLAB [11], which uses matrices A,B,Q, and R to find P via the Ricatti equation [6] and solve for K [5].

<img src="/pics/equation3.png"  width="600" height="500">

Matrices A and B are found by modeling the system’s motion using the Lagrange equations [7][8], then linearizing the results into the state space equations [9].

<img src="/pics/equation4.png"  width="600" height="150">

<img src="/pics/equation5.png"  width="600" height="900">

The values in the state-cost weighted matrix Q determine the level of emphasis placed on minimizing the state’s deviations in the cost function. The Q matrix for this system places equal weight to the Euler angle, angular velocity, and motor speed state, and a much smaller weight (0.001) to the motor position. This means that it is much less important for the motor position to match the desired state than it is for the Euler angle, angular velocity, and motor speed state. The control weighted matrix R equals one, which means that minimizing the state’s deviations is equally as important as minimizing the control effort.

<img src="/pics/equation6.png"  width="600" height="400">

Plugging the four matrices into the lqr() function [4], the theoretical K matrix was determined. By conducted a series of experiments to fine-tune the K values until get the final K matrix [11] provided stable performance. This process allowed us to identify a more effective set of feedback gains that ultimately improved the system's stability and control accuracy. [About LQR calculation using MATLAB](https://github.com/pxmkv/Balance-Tech/tree/main/main_LQR)

<img src="/pics/equation7.png"  width="600" height="60">

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
The LQR controller takes 4 parameters: Euler Angle, Angular velocity, reaction wheel actual speed and reaction wheel position, then calculates the PWM output. 

*Basic structure:

**CPU1:** 

  Calls Kalman_filter() function (process data)

  Calculates the motor speed and position using the encoder data

  Computes the control signal (pwm) based on the LQR control gains (K1-K4)

  The reaction wheel motor is controlled by M_Motor() function

**CPU0:**

  Read Game Controller input using  [Bluepad32](https://github.com/ricardoquesada/bluepad32/blob/main/docs/plat_arduino.md) library
  and mapping joystick reading value to the drivetrain motor and servo motor


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
  

