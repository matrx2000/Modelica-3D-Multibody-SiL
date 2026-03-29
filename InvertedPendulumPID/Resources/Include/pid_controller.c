/*============================================================================
 * pid_controller.c - Discrete PID Controller for Inverted Pendulum
 *
 * This is the kind of C code you'd find running on a real MCU controlling
 * an inverted pendulum via a linear motor. The algorithm is identical to
 * what you'd deploy on an STM32, Arduino, or any embedded platform.
 *
 * Integration with Modelica:
 *   OpenModelica compiles this file and links it into the simulation.
 *   The Modelica model calls pid_controller_step() at discrete time
 *   intervals (e.g., every 5 ms), just like a real timer interrupt would.
 *============================================================================*/

#include "pid_controller.h"

void pid_controller_step(
    double angle,
    double angle_setpoint,
    double angular_rate,
    double integral_in,
    double dt,
    double Kp,
    double Ki,
    double Kd,
    double force_max,
    double integral_max,
    double *force_out,
    double *integral_out)
{
    /*----------------------------------------------------------------------
     * 1. Compute error
     *    Positive error = pendulum tilted in positive direction
     *    The controller must push the cart in the SAME direction to
     *    "catch" the falling pendulum (direct-acting controller).
     *----------------------------------------------------------------------*/
    double error = angle - angle_setpoint;

    /*----------------------------------------------------------------------
     * 2. Proportional term
     *    Immediate response proportional to how far off we are.
     *----------------------------------------------------------------------*/
    double P_term = Kp * error;

    /*----------------------------------------------------------------------
     * 3. Integral term with anti-windup
     *    Accumulates error over time to eliminate steady-state offset.
     *    Clamping prevents windup when the actuator is saturated —
     *    critical on real hardware where the motor has finite force.
     *----------------------------------------------------------------------*/
    double integral = integral_in + error * dt;

    /* Anti-windup: clamp the integral accumulator */
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < -integral_max) {
        integral = -integral_max;
    }

    double I_term = Ki * integral;

    /*----------------------------------------------------------------------
     * 4. Derivative term (derivative-on-measurement)
     *    Uses the measured angular rate directly instead of differencing
     *    the error signal. This avoids "derivative kick" when the
     *    setpoint changes — a standard best practice in embedded control.
     *
     *    On a real MCU, angular_rate would come from:
     *      - A gyroscope (IMU like MPU6050)
     *      - Differentiated encoder signal with filtering
     *      - A Kalman/complementary filter output
     *----------------------------------------------------------------------*/
    double D_term = Kd * angular_rate;

    /*----------------------------------------------------------------------
     * 5. Sum PID terms and apply output saturation
     *    The saturation mimics the physical force limit of the linear
     *    motor. On real hardware this prevents commanding current beyond
     *    the motor driver's rating.
     *----------------------------------------------------------------------*/
    double output = P_term + I_term + D_term;

    /* Output saturation */
    if (output > force_max) {
        output = force_max;

        /* Back-calculation anti-windup: don't integrate further
           in the direction of saturation */
        if (error > 0.0) {
            integral = integral_in;  /* undo this step's integration */
        }
    } else if (output < -force_max) {
        output = -force_max;

        if (error < 0.0) {
            integral = integral_in;
        }
    }

    /*----------------------------------------------------------------------
     * 6. Write outputs
     *    On a real MCU, force_out would be converted to a PWM duty cycle
     *    or a DAC voltage to drive the motor controller.
     *----------------------------------------------------------------------*/
    *force_out = output;
    *integral_out = integral;
}
