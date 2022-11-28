
//----------------MOTOR CONTROL PWM + PID----------------//

// DriverLib Includes
#include <driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>


#define TICKPERIOD      1000


// Global counter variable //
uint32_t SR04IntTimes;
uint32_t leftNotchesDetected;
uint32_t rightNotchesDetected;


uint32_t turning;
uint32_t forward;

uint32_t startNotches = 0;


// Initializes desired and P I D values for PID controller //
uint32_t desiredNotches = 15;           // Sets desired distance.
float kp = 49;                       // Proportional gain.
float ki = 9;                       // Integral gain.
float kd = 0.003;                       // Derivative gain.
//11,11,0.00225

// Initializes error values for PID controller //
float int_errorleft = 0;
float prev_errorleft = 0;
float int_errorright = 0;
float prev_errorright = 0;
float pidOutput = 0;
float dutycycleleft=60;
float dutycycleright=60;


// Run controller at 2 MHz //
float dt = 0.002;


// Timer_A UpMode Configuration Parameter //
const Timer_A_UpModeConfig upConfig =
{
     TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
     TIMER_A_CLOCKSOURCE_DIVIDER_3,          // SMCLK/3 = 1MHz
     TICKPERIOD,                             // 1000 tick period
     TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
     TIMER_A_DO_CLEAR                        // Clear value
};

//30k period left follow right
// Timer_A PWM Configuration Parameter //
// Right Wheel! //
Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        6000
};


// Timer_A PWM Configuration Parameter //
// Left Wheel! //
Timer_A_PWMConfig pwmConfig2 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        6000
};



int main(void)
 {

    leftNotchesDetected = 0;
    rightNotchesDetected = 0;

    // Halting the watchdog //
    WDT_A_holdTimer();

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    //----------------RIGHT PWM----------------//

    // Configuring P4.4 and P4.5 as Output and P2.4 as peripheral output for PWM //
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configuring P2.4 for right interrupt //
    GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN4);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN4);

    // Configuring P2.6 for right encoder //
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

    //----------------LEFT PWM----------------//

    // Configuring P4.0 and P4.2 as Output and P2.5 as peripheral output for PWM //
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configuring P2.5 for left interrupt //
    GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);

    // Configuring P2.7 for left encoder //
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7);



    //----------------TIMER----------------//

    // Configuring Timer_A0 for Up Mode //
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);

    // Enabling interrupts and starting the timer //
    Interrupt_enableInterrupt(INT_TA1_0);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);


    ////Timer_A_stopTimer(TIMER_A0_BASE);
    ////Timer_A_clearTimer(TIMER_A0_BASE);


    // Configuring Timer_A to have a period of approximately 80ms and an initial duty cycle of 10% of that (1000 ticks) //
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);


    // Enabling interrupts and starting the watchdog timer //
    //Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_PORT2);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    //turnRight();
    moveForward();

    // Sleeping when not in use  //
    while (1)
    {
        PCM_gotoLPM0();
    }
}

void stop(void){
    //stop motor
    pwmConfig.dutyCycle = 0;
    pwmConfig2.dutyCycle = 0;
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);
    turning = 0;
    forward = 0;


}
void turnRight(void){
    //stop motor
    stop();
    turning = 1;
    //change direction of wheel
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);

    pwmConfig.dutyCycle = 5000;
    pwmConfig2.dutyCycle = 5000;
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);

}
void turnLeft(void){
    //stop motor
    stop();
    turning = 1;
    //change direction of wheel
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);

    pwmConfig.dutyCycle = 3000;
    pwmConfig2.dutyCycle = 3000;
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);


}

void moveForward(void){
    forward = 1;
    pwmConfig.dutyCycle = 6000;
    pwmConfig2.dutyCycle = 6000;
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);

}

float getPIDOutputLeft()
{

    float error = 0;
    float error_dot = 0;

    error =  (float)desiredNotches - (float)leftNotchesDetected;
    printf("%d %d\n", desiredNotches, leftNotchesDetected);

    // Compute integrated error //
    int_errorleft = int_errorleft + dt*error;


    // Anti-wind up routine //
    if (abs(int_errorleft) > 1.5)
    {
        if (int_errorleft > 0)
        {
            int_errorleft = 1.5;
        }
        else
        {
            int_errorleft = -1.5;
        }
    }
    //printf("int_error = %.2f\t", int_error);
    // Compute derivative of error //
    error_dot = (error - prev_errorleft)/dt;

    // Compute PID control //
    pidOutput = (kp*error + ki*int_errorleft + kd*error_dot);
    if (leftNotchesDetected == 0){pidOutput = 0;}
    if (error == 0){pidOutput = 0;}

    //printf("error = %.f\t Output Left = %d\tOutput Right = %d\tDC Left = %d\tPID Left = %.2f\n", error, leftNotchesDetected,rightNotchesDetected ,pwmConfig2.dutyCycle,pidOutput);
    printf("Left Error = %.2f\t DC Left = %d\t PID Left = %.2f\n",error,pwmConfig2.dutyCycle,pidOutput);
    // Save last error value for next derivative computation //
    prev_errorleft = error;

    // Print current control inputs and states to user //

    return pidOutput;
}

float getPIDOutputRight()
{

    float error = 0;
    float error_dot = 0;

    error =  (float)desiredNotches - (float)rightNotchesDetected;

    // Compute integrated error //
    int_errorright = int_errorright + dt*error;


    // Anti-wind up routine //
    if (abs(int_errorright) > 1.5)
    {
        if (int_errorright > 0)
        {
            int_errorright = 1.5;
        }
        else
        {
            int_errorright = -1.5;
        }
    }
    // Compute derivative of error //
    error_dot = (error - prev_errorright)/dt;

    // Compute PID control //
    pidOutput = (kp*error + ki*int_errorright + kd*error_dot);
    if (rightNotchesDetected == 0){pidOutput = 0;}
    if (error == 0){pidOutput = 0;}

    // Save last error value for next derivative computation //
    prev_errorright = error;

    // Print current control inputs and states to user //
    //printf("error = %.f\tOutput Right = %d\tDC Right = %d\tPID Right = %.2f\n", error, rightNotchesDetected,pwmConfig.dutyCycle,pidOutput);

    printf("Right Error = %.2f\t Output Right = %d\t DC Right = %d\t PID Right = %.2f\n ",error ,rightNotchesDetected,pwmConfig.dutyCycle,pidOutput);

    return pidOutput;
}

// TA0 ISR - This ISR will call getPIDOutput() (which will calculate the error values) and adjust the duty cycle //
void TA1_0_IRQHandler(void)
{

    /* Increment global variable (count number of interrupt occurred) */
    SR04IntTimes++;

    if (SR04IntTimes >= 20000)
    {
        // Initializes variable to store result of getPIDoutput() function //
            float leftControl = 0;
            float rightControl = 0;

            // Gets the PID control output which is used to set the duty cycle //
            leftControl = getPIDOutputLeft();
            rightControl = getPIDOutputRight();

            // left //
            // Set duty cycle based on control input //
/*
            if (leftControl != 0)
            {

                if(pwmConfig2.dutyCycle <= 10000 && pwmConfig2.dutyCycle >= 1000)
                {
                    pwmConfig2.dutyCycle += leftControl;
                }
                else if(pwmConfig2.dutyCycle > 10000){
                    pwmConfig2.dutyCycle = 10000;
                }
                else if(pwmConfig2.dutyCycle < 1000)
                    pwmConfig2.dutyCycle = 1000;
            }
            if (rightControl != 0)
                       {
                           if(pwmConfig.dutyCycle <= 10000 && pwmConfig.dutyCycle >= 1000)
                           {
                               pwmConfig.dutyCycle += rightControl;
                           }
                           else if(pwmConfig.dutyCycle > 10000){
                               pwmConfig.dutyCycle = 10000;
                           }
                           else if(pwmConfig.dutyCycle < 1000)
                               pwmConfig.dutyCycle = 1000;
                       }

*/

            if(turning){

                if(startNotches == 0){
                    startNotches = rightNotchesDetected;

                }
                if(rightNotchesDetected >= startNotches+40){
                    turning = 0;
                    startNotches= 0;

                    //reset direction
                    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
                    stop();
                }

            }
            else if(forward){
                if (leftControl != 0)
                {
                    if(pwmConfig2.dutyCycle <= 10000 && pwmConfig2.dutyCycle >= 1000)
                    {
                        pwmConfig2.dutyCycle += leftControl;
                    }
                    else if(pwmConfig2.dutyCycle > 10000){
                        pwmConfig2.dutyCycle = 10000;
                    }
                    else if(pwmConfig2.dutyCycle < 1000)
                        pwmConfig2.dutyCycle = 1000;
                }
                if (rightControl != 0)
                   {
                       if(pwmConfig.dutyCycle <= 10000 && pwmConfig.dutyCycle >= 1000)
                       {
                           pwmConfig.dutyCycle += rightControl;
                       }
                       else if(pwmConfig.dutyCycle > 10000){
                           pwmConfig.dutyCycle = 10000;
                       }
                       else if(pwmConfig.dutyCycle < 1000)
                           pwmConfig.dutyCycle = 1000;
                   }
                // Reset global variable //
                rightNotchesDetected = 0;
                leftNotchesDetected = 0;
                SR04IntTimes = 0;


                Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
                Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);
            }



    }

    // Clear interrupt flag //
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

// Port2 ISR - This ISR will count the number of notches detected as the wheel turns //
void PORT2_IRQHandler(void)
{
    uint32_t status;

    status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);

    // RIGHT //
    if (status & GPIO_PIN6)
    {
        // Increment global variable (count number of notches detected) //
        rightNotchesDetected++;
        //printf("R%d\n", rightNotchesDetected);
    }

    // LEFT //
    if (status & GPIO_PIN7)
    {
        // Increment global variable (count number of notches detected) //
        leftNotchesDetected++;
        //printf("L%d\n", leftNotchesDetected);

    }



    GPIO_clearInterruptFlag(GPIO_PORT_P2, status);
}


