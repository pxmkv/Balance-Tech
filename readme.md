![img](https://i1.wp.com/www.esp32learning.com/wp-content/uploads/2018/12/MH-ET_LIVE_D1_mini_ESP32_pinout.png?resize=696%2C479)

M_Motor :

ENA ENB ENV DIR PWM PWS GND VCC

M_Encoder GPIO 19, 23

​	PWM : GPIO 26

​	PWS  : GPIO 18

​	DIR    : GPIO 5

Drive Motor

​	GPIO 35, 33

### Installing Libraries

Open your Arduino IDE and go to **Sketch** > **Include Library** > **Manage Libraries**. The Library Manager should open.

Type “**adafruit mpu6050**” on the search box and install the library.

Type “**ESP32Encoder**” on the search box and install the library.

Then, search for “**Adafruit Unified Sensor**”. Scroll all the way down to find the library and install it.

Finally, search for “**Adafruit Bus IO**” and install it

After installing the libraries, restart your Arduino IDE.	

