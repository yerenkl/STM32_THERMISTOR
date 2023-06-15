# STM32_THERMISTOR
Objective of this project is to design a temperature controller similar to those
used in industrial settings. These controllers usually include a display for displaying
the set-point, buttons for adjusting it, a temperature sensor input, and a control
algorithm. The controller will function as both a heating and cooling controller.

Steps that are followed this project are as follows:
- The temperature controller has a default setpoint of 22oC and the
measured temperature is converted to oC with a precision of 0.125oC.
- Different colored LEDs are used to indicate the intensity of the heater
or cooler. The heater is represented by a red LED, and the cooler by a
blue LED.
- PWM frequency is 1kHz and a resolution is 100 steps (ARR value).
- Baud rate is115200 and 8N1 configuration is used to send messages
such as "Heating", "Cooling", PWM duty ratio, and current
temperature.
- The buttons increase/decrease the setpoint temperature by 0.5 oC
respectively.
- Sampling time is set to 0.5 seconds, buttons are responding only when
they are released.
- The gain of the controller is 5 PWM counts for every 0.125oC
temperature error.

Red LED->A0

Blue LED->A1

Saturation LED->A2

Button DOWN->B12

Button UP->B13

UART Connections->A9-A10

I2C Connections-> B6-B7

![image](https://github.com/yerenkl/STM32_THERMISTOR/assets/61627684/73a5440b-cdbb-48d9-a211-50756cdfe4a7)


