within InvertedPendulumPID;

function SwingUpFunction "Modelica wrapper for the external C swing-up + balance controller"
  /*--------------------------------------------------------------------------
   * Bridge between Modelica and the C swing-up controller.
   * Implements a two-mode state machine:
   *   Mode 0: Energy-based swing-up (Astrom & Furuta)
   *   Mode 1: PID balance with position feedback
   *
   * The mode switches automatically based on how close the pendulum is to
   * upright. On a real MCU, this would be a state machine in a timer ISR.
   *--------------------------------------------------------------------------*/

  /* === Inputs from sensors / simulation === */
  input Real angle "Pendulum angle [rad], 0 = upright, pi = hanging";
  input Real angular_rate "Pendulum angular velocity [rad/s]";
  input Real cart_pos "Cart position [m]";
  input Real cart_vel "Cart velocity [m/s]";

  /* === Controller state === */
  input Real integral_in "PID integral accumulator from previous step [rad*s]";
  input Real mode_in "Controller mode: 0 = swing-up, 1 = balance";

  /* === Timing === */
  input Real dt "Control loop sample period [s]";

  /* === Swing-up parameters === */
  input Real k_swing "Swing-up gain [N/J]";
  input Real m_pend "Pendulum bob mass [kg]";
  input Real L_pend "Pendulum length to center of mass [m]";
  input Real g_accel "Gravitational acceleration [m/s^2]";

  /* === PID parameters === */
  input Real Kp "Proportional gain [N/rad]";
  input Real Ki "Integral gain [N/(rad*s)]";
  input Real Kd "Derivative gain [N*s/rad]";
  input Real force_max "Actuator force saturation [N]";
  input Real integral_max "Anti-windup integral limit [rad*s]";

  /* === Position feedback parameters === */
  input Real Kp_pos "Position proportional gain [rad/m]";
  input Real Kd_pos "Position derivative gain [rad*s/m]";
  input Real x_ref "Desired cart position [m]";
  input Real setpoint_max "Max angle setpoint from position loop [rad]";

  /* === Switching === */
  input Real switch_angle "Angle threshold to switch to balance mode [rad]";

  /* === Outputs === */
  output Real force_out "Force command to linear motor [N]";
  output Real integral_out "Updated PID integral accumulator [rad*s]";
  output Real mode_out "Updated controller mode (0 or 1)";

  external "C" swing_up_controller_step(
    angle, angular_rate, cart_pos, cart_vel,
    integral_in, mode_in, dt,
    k_swing, m_pend, L_pend, g_accel,
    Kp, Ki, Kd, force_max, integral_max,
    Kp_pos, Kd_pos, x_ref, setpoint_max,
    switch_angle,
    force_out, integral_out, mode_out)
    annotation(
      Include = "#include \"swing_up_controller.c\"",
      IncludeDirectory = "modelica://InvertedPendulumPID/Resources/Include"
    );

  annotation(Documentation(info = "<html>
    <p>Calls the external C function <code>swing_up_controller_step()</code> which
       implements a two-mode controller:</p>
    <ul>
      <li><b>Mode 0 (Swing-up):</b> Energy-based controller that pumps energy
          into the pendulum by rocking the cart until it reaches upright.</li>
      <li><b>Mode 1 (Balance):</b> PID controller with position feedback,
          identical to <code>pid_controller_step()</code> plus cart centering.</li>
    </ul>
    <p>Mode switching is automatic based on pendulum angle.</p>
    <p>The C source is in <code>Resources/Include/swing_up_controller.c</code>.</p>
    </html>"));
end SwingUpFunction;
