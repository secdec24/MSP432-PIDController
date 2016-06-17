#include "driverlib.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

// START USER INPUT
// Sets desired temperature in degrees Fahrenheit.
float desiredTemp = ;
// Proportional gain.
float kp = ;
// Integral gain.
float ki = ;
// Derivative gain.
float kd = ;
// END USER INPUT

// Sets the flag to control the rate at which the ADC measurements are taken.
volatile uint8_t flag = 0;

// Initializes result for ADC14_getResult();
volatile uint_fast16_t ADCResult = 0;

// Initializes variable to store output temperature from ADC conversion.
volatile float outputTemp = 0.0;

// Initializes error values for PID controller.
float int_error = 0;
float prev_error = 0;
float pidOutput = 0;

// Run controller at 4 MHz.
float dt = 0.250;

// Initializes the duty cycle to 10% of the PWM period.
volatile float dutyCycle = periodPWM * 0.01;

// Configures timer in Up mode.
const Timer_A_UpModeConfig upConfig_0 = {
    // Tie Timer A to SMCLK
    TIMER_A_CLOCKSOURCE_SMCLK,
    // Increment counter every 64 clock cycles: 1 MHz/64 = 15625 Hz
    TIMER_A_CLOCKSOURCE_DIVIDER_64,
    // Period of Timer A, generates interrupt every time we get to 15625
    1562,
    // Disable Timer A rollover interrupt
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    // Enable Capture Compare interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
    //Clear counter upon initialization
    TIMER_A_DO_CLEAR
};

int main(void) {
    WDT_A_holdTimer();

    // Initializes variable to store result of getPIDoutput() function.
    float control = 0;

    //----------------CLOCK----------------//
    // Set DCO to 1 MHz.
    MAP_CS_setDCOFrequency(1E+6);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    //----------------OUTPUT PINS----------------//
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);

    //----------------PWM----------------//
    // Sets the PWM period.
    TA0CCR0 = periodPWM;
    // Initializes PWM duty cycle to 100.
    TA0CCR1 = dutyCycle;
    // Reset/Set Output Mode
    TA0CCTL1 = OUTMOD_7;
    // SMCLK, Up Mode, Clear TACLR
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;

    //----------------TIMER----------------//
    // Configures Timer A1.
    MAP_Timer_A_configureUpMode(TIMER_A1_MODULE, &upConfig_0);
    // Enables Timer A interrupt.
    MAP_Interrupt_enableInterrupt(INT_TA1_0);
    // Starts Timer A1
    MAP_Timer_A_startCounter(TIMER_A1_MODULE, TIMER_A_UP_MODE);

    //----------------ADC14----------------//
    // Enable ADC Module
    MAP_ADC14_enableModule();
    // Set P5.5 (A0) to be ADC functional.
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);
    // Set to 10 bit resolution.
    MAP_ADC14_setResolution(ADC_10BIT);
    // Tie to SMCLK with no divider.
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);
    // Place result of conversion in ADC_MEM0 register.
    // Sets repeat mode to false: each conversion will be triggered manually.
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, false);
    // Configures memory conversion and voltage range.
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);
    // Configures sample timer; triggers each conversion manually.
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    // Enables Conversion
    // Sets ENC bit.
    MAP_ADC14_enableConversion();
    // Toggles conversion trigger to start the conversion.
    MAP_ADC14_toggleConversionTrigger();

    while(1) {
        // Flag is set to 1 every dt sec.
        if (flag) {
            //If there are no conversions in progress.
            if (!MAP_ADC14_isBusy()) {
                // Get ADC conversion result.
                ADCResult = MAP_ADC14_getResult(ADC_MEM0);
                outputTemp = ADCResult * (3.3/1023)/0.01;

                // Toggles conversion trigger to start the conversion.
                MAP_ADC14_toggleConversionTrigger();

                // Gets the PID control output which is used to set the duty cycle.
                control = getPIDOutput();

                // Set duty cycle based on control input.
                if (control > 0 && control < 1) {
                    TA0CCR1 = control * 10000;
                } else if (control > 1) {
                    // Limit to 100% duty cycle.
                    TA0CCR1 = 10000;
                } else if (control < 0) {
                    // Limit to 0% duty cycle.
                    TA0CCR1 = 0;
                }

                // Sets the flag to zero.
                flag = 0;
            }
        }
    }
}

void timerA_isr() {
    // Clears interrupt.
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // Set flag to run control cycle.
    flag = 1;
}

float getPIDOutput() {
    // Initializes errors.
    float error = 0;
    float error_dot = 0;

    // Compute error on scale normalized by total temperature range of 0-150 degrees.
    error = (desiredTemp - outputTemp)/150.0;

    // Compute integrated error.
    int_error = int_error + dt*error;

    // Anti-wind up routine.
    if (abs(int_error) > 1.5) {
        if (int_error > 0) {
            int_error = 1.5;
        } else {
            int_error = -1.5;
        }
    }

    // Compute derivative of error.
    error_dot = (error - prev_error)/dt;

    // Compute PID control.
    pidOutput = (kp*error + ki*int_error + kd*error_dot);

    // Save last error value for next derivative computation.
    prev_error = error;

    // Print current control inputs and states to user.
    printf("T_des = %.2f\t", desiredTemp);
    printf("T = %.2f\t", outputTemp);
    printf("DC = %.2d\t", TA0CCR1);
    printf("PID = %.2f\n", pidOutput);

    return pidOutput;
}
