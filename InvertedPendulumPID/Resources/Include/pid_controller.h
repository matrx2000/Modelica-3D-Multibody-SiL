/*============================================================================
 * pid_controller.h - Discrete PID Controller for Inverted Pendulum
 *
 * Target: Generic MCU (ARM Cortex-M, AVR, or similar)
 *
 * This module implements a discrete-time PID controller as it would run
 * inside a microcontroller's timer interrupt. Features:
 *   - Anti-windup via integral clamping
 *   - Derivative-on-measurement (avoids derivative kick on setpoint change)
 *   - Output saturation (mimics real actuator limits)
 *
 * In a real MCU project, this would be called from a timer ISR at a fixed
 * rate (e.g., 5 ms). Here it is called from the Modelica simulation via
 * the external "C" interface, sampled at the same fixed rate.
 *============================================================================*/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/*
 * pid_controller_step - Single PID computation step
 *
 * Called once per control cycle. On a real MCU this would live in a
 * timer ISR or an RTOS task triggered by a hardware timer.
 *
 * Inputs:
 *   angle          - measured pendulum angle [rad], 0 = upright vertical
 *   angle_setpoint - desired angle [rad], typically 0 for balancing
 *   angular_rate   - measured angular velocity [rad/s] (from gyro or encoder)
 *   integral_in    - integral accumulator state from previous step [rad*s]
 *   dt             - control loop period [s]
 *   Kp             - proportional gain [N/rad]
 *   Ki             - integral gain [N/(rad*s)]
 *   Kd             - derivative gain [N*s/rad]
 *   force_max      - actuator saturation limit [N] (symmetric +/-)
 *   integral_max   - anti-windup: max allowed integral accumulation [rad*s]
 *
 * Outputs (via pointers):
 *   force_out      - computed force command to linear motor [N]
 *   integral_out   - updated integral accumulator for next step [rad*s]
 */
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
    double *integral_out
);

#endif /* PID_CONTROLLER_H */
