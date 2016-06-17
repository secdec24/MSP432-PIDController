# MSP432-PIDController

PID controller for a thermal system using the TI MSP432 microcontroller. The MSP432 uses the feedback controller to change the temperature inside a canister to the desired value by using duty cycles as the contol input. A heat sensor in the canister provides feedback to the MSP432.

## MSP432 Components Used

* TimerA1_Module 0 `TA1_0`
* PWM `Pin 2.4`: Used to apply current signal to a heating element (resistor in canister) to control inside the canister. A MOSFET gate is used to connect the PWM port in the microcontroller to the resistor. By using the transistor, the current flow through the resistor can be changed by adjusting the duty cycle.
* ADC `Pin 5.5`

## Instructions

1. The user should set the desired temperature as well as the gains for the system at the beginning of the code. A model of the system should be created to determine appropriate gains.

```c
// TODO: START USER INPUT
// Sets desired temperature in degrees Fahrenheit.
float desiredTemp = ;
// Proportional gain.
float kp = ;
// Integral gain.
float ki = ;
// Derivative gain.
float kd = ;
// TODO: END USER INPUT
```
