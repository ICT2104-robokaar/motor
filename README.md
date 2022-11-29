# motor
PID stands for proportional, integral, derivative

Proportional(Present error)
Integral(Past error)
Derivative(Future error)

P gain can reduce Steady state error, but it will induce Overshoot.
Steady state error - difference between the desired output and the actual one.
I gain can reduce Steady state error, but it will increase Overshoot.
D gain can reduce Overshoot, but it has no change on Steady state error.

PID Tuning
- Therefore, We did trial and error.

PID Controller - Calculates the difference between the desired notches and actual notches(error values) and adjust the duty cycle.

Left motor:
Error = desiredNotches – leftNotchesDetected;

Right motor:
Error = desiredNotches – rightNotchesDetected;

Formula to calculate integral error:
int_error = int_error + dt*error;

Formula to calculate derivative error:
error_der = (error - prev_error) / dt;

Formula to calculate PID output:
pidOutput = (kp*error + ki*int_error + kd*error_der);

Console output:
![photo6066664932671337826](https://user-images.githubusercontent.com/77524281/204537682-142af1ad-d193-441e-95b3-7e3842d93ade.jpg)


