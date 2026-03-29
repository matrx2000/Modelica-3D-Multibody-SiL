/*============================================================================
 * swing_up_controller.c - Energy-Based Swing-Up + PID Balancing Controller
 *
 * Two-mode controller for inverted pendulum:
 *
 *   SWING-UP (mode 0):
 *     Uses the Astrom & Furuta energy-based method. The idea: compute the
 *     pendulum's total energy relative to the upright equilibrium. Apply a
 *     force proportional to the energy deficit, in the direction that adds
 *     energy (same direction as omega * cos(theta)). This naturally swings
 *     the pendulum in larger arcs until it reaches the top.
 *
 *   BALANCE (mode 1):
 *     Standard PID on angle with position feedback outer loop.
 *     Identical to pid_controller_step() plus the position feedback logic.
 *
 *   SWITCHING:
 *     When |angle| < switch_angle, transition from swing-up to balance.
 *     Once in balance mode, if the pendulum falls beyond switch_angle,
 *     fall back to swing-up mode. This makes the controller robust to
 *     disturbances that knock the pendulum away from upright.
 *
 * On a real MCU, this would be a state machine in a timer ISR. The mode
 * variable is the state, passed in/out via Modelica discrete variables.
 *============================================================================*/

#include "swing_up_controller.h"
#include <math.h>

/* Helper: clamp a value to [-limit, +limit] */
static double clamp(double val, double limit)
{
    if (val > limit) return limit;
    if (val < -limit) return -limit;
    return val;
}

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
    double *mode_out)
{
    /*----------------------------------------------------------------------
     * Normalize angle to [-pi, pi] range
     * The MultiBody revolute joint can accumulate angle beyond [-pi, pi]
     * if the pendulum spins. Normalize so the energy calculation and
     * switching logic work correctly.
     *----------------------------------------------------------------------*/
    double theta = fmod(angle, 2.0 * M_PI);
    if (theta > M_PI) theta -= 2.0 * M_PI;
    if (theta < -M_PI) theta += 2.0 * M_PI;

    double abs_theta = fabs(theta);

    /*----------------------------------------------------------------------
     * Mode switching logic
     *----------------------------------------------------------------------*/
    int mode = (int)mode_in;

    if (mode == 0 && abs_theta < switch_angle) {
        /* Near upright: switch to PID balance */
        mode = 1;
    } else if (mode == 1 && abs_theta > switch_angle * 1.5) {
        /* Fallen too far from upright: fall back to swing-up */
        mode = 0;
    }

    if (mode == 0) {
        /*------------------------------------------------------------------
         * SWING-UP MODE: Energy-based controller (Astrom & Furuta)
         *
         * Pendulum energy relative to upright equilibrium:
         *   E = 0.5 * m * L^2 * omega^2 + m * g * L * (cos(theta) - 1)
         *
         * At upright (theta=0, omega=0): E = 0  (reference)
         * At hanging (theta=pi, omega=0): E = -2*m*g*L
         *
         * We want E -> 0 (upright energy level).
         *
         * Control law:
         *   F = k_swing * E * omega * cos(theta)
         *
         * This applies force in the direction that increases energy when
         * E < 0 (below upright energy) and reduces energy when E > 0
         * (overshooting). The cos(theta) factor ensures correct sign
         * mapping between pendulum angular velocity and cart force.
         *------------------------------------------------------------------*/
        double E = 0.5 * m_pend * L_pend * L_pend * angular_rate * angular_rate
                   + m_pend * g_accel * L_pend * (cos(theta) - 1.0);

        double output = k_swing * E * angular_rate * cos(theta);

        /*--------------------------------------------------------------
         * Initial kick: when the pendulum is near the bottom and barely
         * moving, the energy controller produces ~zero force (because
         * omega ≈ 0). Give it a push to get things started.
         *--------------------------------------------------------------*/
        if (fabs(angular_rate) < 0.5 && fabs(cos(theta) + 1.0) < 0.3) {
            output += force_max * 0.4;
        }

        /* Gentle cart centering during swing-up to keep cart on track */
        output += -0.5 * (cart_pos - x_ref) - 0.3 * cart_vel;

        /* Saturate */
        output = clamp(output, force_max);

        *force_out = output;
        *integral_out = 0.0;   /* Reset PID integral for clean handoff */
        *mode_out = (double)mode;

    } else {
        /*------------------------------------------------------------------
         * BALANCE MODE: PID with position feedback
         * (Same algorithm as pid_controller_step + position outer loop)
         *------------------------------------------------------------------*/

        /* Outer loop: position feedback -> angle setpoint */
        double angle_setpoint = clamp(
            -Kp_pos * (cart_pos - x_ref) - Kd_pos * cart_vel,
            setpoint_max);

        /* PID on angle error */
        double error = theta - angle_setpoint;

        /* Proportional */
        double P_term = Kp * error;

        /* Integral with anti-windup */
        double integral = integral_in + error * dt;
        integral = clamp(integral, integral_max);
        double I_term = Ki * integral;

        /* Derivative on measurement */
        double D_term = Kd * angular_rate;

        /* Sum and saturate */
        double output = P_term + I_term + D_term;

        /* Back-calculation anti-windup at saturation */
        if (output > force_max) {
            output = force_max;
            if (error > 0.0) integral = integral_in;
        } else if (output < -force_max) {
            output = -force_max;
            if (error < 0.0) integral = integral_in;
        }

        *force_out = output;
        *integral_out = integral;
        *mode_out = (double)mode;
    }
}
