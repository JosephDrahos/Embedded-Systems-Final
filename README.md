# Embedded-Systems-Final
Embedded systems final project using embedded C in MCUXpresso, a FRDM-KL25Z microcontroller, and components from a RexQualis starter kit. 

Components needed form RexQualis starter kit: breadboard, wires, potentiometer, 2 photoresistors, LCD display, servo motor, dc motor, ultrasonic sensor, 4x4 matrix keypad, L293D chip.

Wire configuration:
Keypad: 
Pin 1 -> PORTD4
Pin 2 -> PORTD5
Pin 3 -> PORTD6
Pin 4 -> PORTD7
Pin 5 -> PORTD3
Pin 6 -> PORTD2
Pin 7 -> PORTD1
Pin 8 -> PORTD0

LCD:
D7 -> PORTC13
D6 -> PORTC12
D5 -> PORTC11
D4 -> PORTC10
EN -> PORTC9
RS -> PORTC8
Vss -> Ground
Vdd -> 5V
Backlight Anode -> 5V
Backlight Cathode -> Ground

Servo Motor:
Orange -> PORTC2 (PWM signal)
Red -> FRDM P3V3/(PORTE21 -> photoresistor1)
Brown -> ground/(PORTE21 -> photresistor2)

DC Motor/L293D:
L293D Pin 1 -> PORTC4
"" Pin 2/8/16 -> 5V
"" Pin 3 -> DC Motor high
"" Pin 4/7 -> Ground
"" Pin 6 -> DC Motor low

UltraSonic Sensor:
GND -> Ground
Echo -> POPRTE233
Trig -> PORTC2
Vcc -> P5V

Potentiometer:
inputs -> Ground and 5V
output -> PORTE20

PuTTY Baud Rate = 115200

Mode explainations:
  Mode 1:
    The position of the servo is controlled by the angle of the potentiometer unless one of the photoresistors is covered using       analog to digital conversion from the potentiometer and pulse width modulation to control the postion of the motor.
  Mode2:
    The angle of the potentiometer controls the speed of the DC motor unless an object is detected within a foot of the ultrasonic     senor. If there is an object the motor will stop. The LED on the board also indicated whether an object is detected or not.
  Mode 3:
    The servo motor will continuously scan back and forth from -90 degreeess to +90 degrees where the speed of the scan is             controlled using PuTTy as an input and communicated with the board using UART.
  Mode 4:
    The postion of the servo motor is determined by the angle entered in the PuTTY terminal. 
  Mode 5:
    The speed of the DC motor is determined by the input from the PuTTY terminal. 

More detail on how each component works and how each element on the board is utilized is found in the EE260_finalproject.pdf report.

The code is not the neatest as there was a strict time constraint and making the 5 modes meet the requirements was the main priority.
