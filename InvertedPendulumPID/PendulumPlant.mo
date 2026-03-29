within InvertedPendulumPID;

block PendulumPlant "Causal (explicit) plant model for the inverted pendulum on cart"
  /*==========================================================================
   * This block encapsulates the pendulum-on-cart dynamics in CAUSAL form
   * (input -> output), suitable for signal-flow / block-diagram wiring.
   *
   * The coupled equations of motion are algebraically solved into explicit
   * form (a = ..., alpha = ...) so they can be evaluated step-by-step.
   *
   * Compare with InvertedPendulumSystem.mo, which writes the same physics
   * as implicit (acausal) equations and lets the DAE solver handle them.
   *==========================================================================*/

  import Modelica.Constants.g_n;

  /* === Parameters === */
  parameter Real M = 1.0   "Cart mass [kg]";
  parameter Real m = 0.2   "Pendulum bob mass [kg]";
  parameter Real L = 0.5   "Pendulum length [m]";
  parameter Real b = 0.1   "Cart friction [N*s/m]";
  parameter Real g = g_n   "Gravitational acceleration [m/s^2]";
  parameter Real theta_0 = 0.15  "Initial pendulum angle [rad]";

  /* === Connectors === */
  Modelica.Blocks.Interfaces.RealInput force "Motor force [N]"
    annotation(Placement(transformation(extent = {{-140, -20}, {-100, 20}})));

  Modelica.Blocks.Interfaces.RealOutput angle "Pendulum angle [rad]"
    annotation(Placement(transformation(extent = {{100, 40}, {140, 80}})));

  Modelica.Blocks.Interfaces.RealOutput angularRate "Pendulum angular velocity [rad/s]"
    annotation(Placement(transformation(extent = {{100, -80}, {140, -40}})));

  Modelica.Blocks.Interfaces.RealOutput cartPosition "Cart position [m]"
    annotation(Placement(transformation(extent = {{100, 80}, {140, 120}})));

  Modelica.Blocks.Interfaces.RealOutput cartVelocity "Cart velocity [m/s]"
    annotation(Placement(transformation(extent = {{100, -120}, {140, -80}})));

  /* === States === */
  Real x(start = 0, fixed = true)      "Cart position [m]";
  Real v(start = 0, fixed = true)      "Cart velocity [m/s]";
  Real theta(start = theta_0, fixed = true)  "Pendulum angle [rad]";
  Real omega(start = 0, fixed = true)  "Pendulum angular velocity [rad/s]";

protected
  Real a    "Cart acceleration [m/s^2]";
  Real alpha  "Pendulum angular acceleration [rad/s^2]";
  Real denom  "Denominator of explicit solution (mass matrix determinant)";

equation
  /*------------------------------------------------------------------------
   * Explicit (causal) form of the equations of motion.
   *
   * Starting from the coupled implicit equations:
   *   (M+m)*a + m*L*alpha*cos(theta) - m*L*omega^2*sin(theta) + b*v = F
   *   m*L*a*cos(theta) + m*L^2*alpha - m*g*L*sin(theta) = 0
   *
   * Algebraically solving the 2x2 system for a and alpha gives:
   *------------------------------------------------------------------------*/

  denom = M + m - m * cos(theta) ^ 2;

  a = (force - m * g * sin(theta) * cos(theta)
       + m * L * omega ^ 2 * sin(theta) - b * v) / denom;

  alpha = (g * sin(theta) - a * cos(theta)) / L;

  // State derivatives
  der(x) = v;
  der(v) = a;
  der(theta) = omega;
  der(omega) = alpha;

  // Outputs
  angle = theta;
  angularRate = omega;
  cartPosition = x;
  cartVelocity = v;

  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Text(extent = {{-80, 40}, {80, -40}}, textString = "Plant"),
      Text(extent = {{-80, 80}, {80, 50}}, textString = "%name")}),
    Documentation(info = "<html>
      <p>Causal (explicit) model of the inverted pendulum on a cart.</p>
      <p>Input: horizontal force on cart. Outputs: pendulum angle and angular rate.</p>
      <p>The coupled equations of motion are solved algebraically into explicit form
         so this block can be used in a signal-flow block diagram.</p>
      </html>"));
end PendulumPlant;
