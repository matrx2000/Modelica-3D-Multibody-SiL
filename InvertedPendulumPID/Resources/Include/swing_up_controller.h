/*============================================================================
 * swing_up_controller.h - Energy-Based Swing-Up + PID Balancing Controller
 *
 * Target: Generic MCU (ARM Cortex-M, AVR, or similar)
 *
 * This module implements a two-mode controller for an inverted pendulum:
 *   Mode 0 (SWING_UP): Energy-based controller that pumps energy into the
 *     pendulum by rocking the cart, swinging it from hanging down to upright.
 *   Mode 1 (BALANCE): Switches to PID balancing once near the top.
 *
 * The switching logic is a simple state machine — the same pattern you'd
 * use in a real MCU firmware with a timer ISR.
 *============================================================================*/

#ifndef SWING_UP_CONTROLLER_H
#define SWING_UP_CONTROLLER_H

/*
 * swing_up_controller_step - Single control step with automatic mode switching
 *
 * Inputs:
 *   angle          - measured pendulum angle [rad], 0 = upright, pi = hanging
 *   angular_rate   - measured angular velocity [rad/s]
 *   cart_pos       - measured cart position [m]
 *   cart_vel       - measured cart velocity [m/s]
 *   integral_in    - PID integral accumulator from previous step [rad*s]
 *   mode_in        - controller mode from previous step (0 = swing-up, 1 = balance)
 *   dt             - control loop period [s]
 *
 *   // Swing-up parameters
 *   k_swing        - swing-up gain [N/J]
 *   m_pend         - pendulum bob mass [kg]
 *   L_pend         - pendulum length to center of mass [m]
 *   g_accel        - gravitational acceleration [m/s^2]
 *
 *   // PID parameters
 *   Kp, Ki, Kd     - PID gains
 *   force_max      - actuator saturation [N]
 *   integral_max   - anti-windup limit [rad*s]
 *
 *   // Position feedback parameters
 *   Kp_pos         - position proportional gain [rad/m]
 *   Kd_pos         - position derivative gain [rad*s/m]
 *   x_ref          - desired cart position [m]
 *   setpoint_max   - max angle setpoint from position loop [rad]
 *
 *   // Switching threshold
 *   switch_angle   - angle threshold for switching to PID [rad]
 *
 * Outputs (via pointers):
 *   force_out      - force command to linear motor [N]
 *   integral_out   - updated PID integral accumulator [rad*s]
 *   mode_out       - updated controller mode (0 or 1)
 */
void swing_up_controller_step(
    double angle,
    double angular_rate,
    double cart_pos,
    double cart_vel,
    double integral_in,
    double mode_in,
    double dt,
    double k_swing,
    double m_pend,
    double L_pend,
    double g_accel,
    double Kp,
    double Ki,
    double Kd,
    double force_max,
    double integral_max,
    double Kp_pos,
    double Kd_pos,
    double x_ref,
    double setpoint_max,
    double switch_angle,
    double *force_out,
    double *integral_out,
    double *mode_out
);

#endif /* SWING_UP_CONTROLLER_H */
