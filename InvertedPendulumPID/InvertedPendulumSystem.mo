within InvertedPendulumPID;

model InvertedPendulumSystem "Inverted pendulum on cart with discrete C-based PID controller"
  /*==========================================================================
   * Full closed-loop simulation:
   *   - Continuous plant: cart + rigid pendulum (2-DOF, coupled dynamics)
   *   - Discrete controller: PID in C, called at fixed sample rate
   *
   * State variables:
   *   x      - cart position [m]
   *   v      - cart velocity [m/s]
   *   theta  - pendulum angle from vertical [rad], 0 = upright
   *   omega  - pendulum angular velocity [rad/s]
   *==========================================================================*/

  import Modelica.Constants.pi;
  import Modelica.Constants.g_n;

  /* ============================= Parameters ============================== */

  // Physical plant
  parameter Real M = 1.0   "Cart mass [kg]";
  parameter Real m = 0.2   "Pendulum bob mass [kg]";
  parameter Real L = 0.5   "Pendulum length (to center of mass) [m]";
  parameter Real b = 0.1   "Cart friction coefficient [N*s/m]";
  parameter Real g = g_n   "Gravitational acceleration [m/s^2]";

  // PID tuning (balance mode)
  parameter Real Kp = 100.0  "Proportional gain [N/rad]";
  parameter Real Ki = 20.0   "Integral gain [N/(rad*s)]";
  parameter Real Kd = 20.0   "Derivative gain [N*s/rad]";
  parameter Real F_max = 50.0       "Actuator force saturation [N]";
  parameter Real integral_limit = 5.0  "Anti-windup integral limit [rad*s]";

  // Position feedback (outer loop, active in balance mode)
  parameter Real Kp_pos = 0.3     "Position proportional gain [rad/m]";
  parameter Real Kd_pos = 0.5     "Position derivative gain [rad*s/m]";
  parameter Real x_ref = 0.0      "Desired cart position [m]";
  parameter Real setpoint_max = 0.1  "Max angle setpoint from position loop [rad] (~5.7 deg)";

  // Swing-up parameters
  parameter Real k_swing = 5.0    "Swing-up gain [N/J]";
  parameter Real switch_angle = 0.3  "Angle threshold to switch to PID [rad] (~17 deg)";

  // Timing
  parameter Real Ts = 0.005  "Controller sample period [s] (200 Hz)";

  // Initial conditions
  parameter Real theta_0 = Modelica.Constants.pi  "Initial pendulum angle [rad] (pi = hanging down)";

  /* ========================= Continuous states =========================== */

  Real x(start = 0, fixed = true)      "Cart position [m]";
  Real v(start = 0, fixed = true)      "Cart velocity [m/s]";
  Real theta(start = theta_0, fixed = true)  "Pendulum angle [rad]";
  Real omega(start = 0, fixed = true)  "Pendulum angular velocity [rad/s]";

  /* ====================== Intermediate variables ========================= */

  Real a   "Cart acceleration [m/s^2]";
  Real alpha  "Pendulum angular acceleration [rad/s^2]";

  /* ========================= Discrete states ============================= */

  discrete Real F(start = 0, fixed = true)  "Motor force command [N]";
  discrete Real integral_state(start = 0, fixed = true)  "PID integral accumulator [rad*s]";
  discrete Real ctrl_mode(start = 0, fixed = true)  "Controller mode: 0 = swing-up, 1 = balance";

  /* ========================= Convenience outputs ========================= */

  Real theta_deg = theta * 180.0 / pi  "Pendulum angle [deg] (for plotting)";
  Real pendulum_tip_x = x + L * sin(theta)  "Pendulum tip x-coordinate [m]";
  Real pendulum_tip_y = L * cos(theta)       "Pendulum tip y-coordinate [m]";

equation
  /*------------------------------------------------------------------------
   * Continuous plant dynamics
   *
   * Equations of motion for cart + rigid pendulum, derived from
   * Lagrangian mechanics. These are coupled — the cart acceleration (a)
   * and pendulum angular acceleration (alpha) appear in both equations.
   * The Modelica DAE solver handles this coupling automatically.
   *------------------------------------------------------------------------*/

  // Kinematic relations
  der(x) = v;
  der(theta) = omega;
  der(v) = a;
  der(omega) = alpha;

  // Coupled equations of motion (Newton-Euler / Lagrangian)
  // Equation 1: horizontal force balance on cart + pendulum system
  (M + m) * a + m * L * alpha * cos(theta) - m * L * omega^2 * sin(theta) + b * v = F;

  // Equation 2: rotational dynamics of pendulum about pivot
  m * L * a * cos(theta) + m * L^2 * alpha - m * g * L * sin(theta) = 0;

  /*------------------------------------------------------------------------
   * Discrete controller (swing-up + PID in C)
   *
   * Fires every Ts seconds, mimicking a real MCU timer interrupt.
   * Between samples, F holds its last value (zero-order hold).
   * The controller automatically switches between swing-up and balance.
   *------------------------------------------------------------------------*/
  when sample(0, Ts) then
    (F, integral_state, ctrl_mode) = SwingUpFunction(
      theta,                    // current angle
      omega,                    // angular rate
      pre(x),                   // cart position
      pre(v),                   // cart velocity
      pre(integral_state),      // previous integral state
      pre(ctrl_mode),           // previous mode
      Ts,                       // sample period
      k_swing,                  // swing-up gain
      m, L, g,                  // plant parameters for energy calc
      Kp, Ki, Kd,              // PID gains
      F_max,                    // actuator saturation
      integral_limit,           // anti-windup limit
      Kp_pos, Kd_pos,          // position feedback gains
      x_ref, setpoint_max,     // position reference
      switch_angle              // switching threshold
    );
  end when;

  annotation(
    experiment(
      StartTime = 0,
      StopTime = 20,
      Tolerance = 1e-6,
      Interval = 0.001
    ),
    Documentation(info = "<html>
      <p>Inverted pendulum on a cart driven by a linear motor with swing-up and
         balance control implemented in C.</p>
      <p>The pendulum starts hanging down (theta_0 = pi). The energy-based swing-up
         controller rocks the cart to pump energy into the pendulum. Once near
         upright, it switches to PID balancing with position feedback.</p>
      </html>"));
end InvertedPendulumSystem;
