within InvertedPendulumPID;

block PIDControllerBlock "Discrete swing-up + PID controller block wrapping the external C code"
  /*==========================================================================
   * Signal-flow wrapper around SwingUpFunction (the C bridge).
   * Reads angle, angular rate, cart position and velocity. Outputs motor force.
   * Executes at fixed sample rate Ts, exactly like a real MCU timer ISR.
   *
   * Mode 0: Energy-based swing-up (Astrom & Furuta)
   * Mode 1: PID balance with position feedback
   * Switching is automatic based on pendulum angle.
   *==========================================================================*/

  import Modelica.Constants.g_n;

  /* === Parameters === */
  // PID tuning (balance mode)
  parameter Real Kp = 100.0  "Proportional gain [N/rad]";
  parameter Real Ki = 20.0   "Integral gain [N/(rad*s)]";
  parameter Real Kd = 20.0   "Derivative gain [N*s/rad]";
  parameter Real F_max = 50.0       "Actuator force saturation [N]";
  parameter Real integral_limit = 5.0  "Anti-windup integral limit [rad*s]";
  parameter Real Ts = 0.005  "Sample period [s]";

  // Position feedback (outer loop, active in balance mode)
  parameter Real Kp_pos = 0.3     "Position proportional gain [rad/m]";
  parameter Real Kd_pos = 0.5     "Position derivative gain [rad*s/m]";
  parameter Real x_ref = 0.0      "Desired cart position [m]";
  parameter Real setpoint_max = 0.1  "Max angle setpoint from position loop [rad] (~5.7 deg)";

  // Swing-up parameters
  parameter Real k_swing = 8.0    "Swing-up gain [N/J]";
  parameter Real switch_angle = 0.3  "Angle threshold to switch to PID [rad] (~17 deg)";

  // Plant parameters (needed for energy calculation in swing-up)
  parameter Real m_pend = 0.2  "Pendulum bob mass [kg]";
  parameter Real L_pend = 0.5  "Pendulum length to center of mass [m]";

  /* === Connectors === */
  Modelica.Blocks.Interfaces.RealInput angle "Measured pendulum angle [rad]"
    annotation(Placement(transformation(extent = {{-140, 20}, {-100, 60}})));

  Modelica.Blocks.Interfaces.RealInput angularRate "Measured angular velocity [rad/s]"
    annotation(Placement(transformation(extent = {{-140, -60}, {-100, -20}})));

  Modelica.Blocks.Interfaces.RealInput cartPosition "Measured cart position [m]"
    annotation(Placement(transformation(extent = {{-140, 60}, {-100, 100}})));

  Modelica.Blocks.Interfaces.RealInput cartVelocity "Measured cart velocity [m/s]"
    annotation(Placement(transformation(extent = {{-140, -100}, {-100, -60}})));

  Modelica.Blocks.Interfaces.RealOutput force "Motor force command [N]"
    annotation(Placement(transformation(extent = {{100, -20}, {140, 20}})));

  /* === Discrete state === */
  discrete Real F_internal(start = 0, fixed = true) "Force output [N]";
  discrete Real integral_state(start = 0, fixed = true) "Integral accumulator [rad*s]";
  discrete Real ctrl_mode(start = 0, fixed = true) "Controller mode: 0 = swing-up, 1 = balance";

equation
  force = F_internal;

  when sample(0, Ts) then
    (F_internal, integral_state, ctrl_mode) = SwingUpFunction(
      angle,                    // measured angle
      angularRate,              // measured angular rate
      pre(cartPosition),        // cart position
      pre(cartVelocity),        // cart velocity
      pre(integral_state),      // previous integral state
      pre(ctrl_mode),           // previous mode
      Ts,                       // sample period
      k_swing,                  // swing-up gain
      m_pend, L_pend, g_n,     // plant parameters for energy calc
      Kp, Ki, Kd,              // PID gains
      F_max,                    // saturation
      integral_limit,           // anti-windup limit
      Kp_pos, Kd_pos,          // position feedback gains
      x_ref, setpoint_max,     // position reference
      switch_angle              // switching threshold
    );
  end when;

  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Text(extent = {{-80, 40}, {80, -40}}, textString = "Swing+PID"),
      Text(extent = {{-80, 80}, {80, 50}}, textString = "%name")}),
    Documentation(info = "<html>
      <p>Discrete-time controller block with two modes:</p>
      <ul>
        <li><b>Swing-up:</b> Energy-based controller that pumps the pendulum up</li>
        <li><b>Balance:</b> PID with position feedback once near upright</li>
      </ul>
      <p>Calls the external C function <code>swing_up_controller_step()</code>.</p>
      </html>"));
end PIDControllerBlock;
