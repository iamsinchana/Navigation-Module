# Navigation-Module
#nstallation and Setup
Hardware:

Connect the components as per the wiring diagram above.

Ensure a properly formatted FAT32 SD Card is inserted into the SD card module.

Software:

Install the following libraries in the Arduino IDE:

Adafruit MPU6050

Adafruit Unified Sensor

SD (Ensure it supports ESP32)

Wire

Upload Code:

Open the provided sketch in the Arduino IDE.

Select the ESP32-S2 Dev Board and the correct port.

Upload the code to the board.

Monitor:

Open the Serial Monitor at 115200 baud rate.

Observe initialization messages and logged data.
#How It Works
Ultrasonic Sensor:

A hardware timer triggers the TRIG pin every 100ms.

Interrupts on the ECHO pin measure the pulse width and calculate the distance.

LED Indicators:

Red LED lights up for detected obstacles.

Yellow LED lights up during obstacle avoidance.

Green LED lights up during forward movement.

MPU6050:

Continuously reads accelerometer and gyroscope data via I2C.

SD Card Logging:

Logs system data, including sensor readings, every 500ms.

FreeRTOS Tasks:

Navigation Task: Manages LED indicators for motion simulation.

Data Logging Task: Writes system status to the SD card.
