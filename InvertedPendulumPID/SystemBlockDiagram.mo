within InvertedPendulumPID;

model SystemBlockDiagram "Block diagram view: PID controller + plant in feedback loop"
  /*==========================================================================
   * Same system as InvertedPendulumSystem, but expressed as a BLOCK DIAGRAM
   * with explicit signal-flow connections.
   *
   * Open this model in OMEdit to see the control loop structure:
   *
   *   +----------------+      force      +----------------+
   *   | PID Controller | -------------> |   Pendulum     |
   *   |     (C code)   |                |    Plant       |
   *   |                | <------------- |                |
   *   +----------------+  angle, omega  +----------------+
   *
   * The plant uses the explicit (causal) form of the equations — the 2x2
   * coupled system is solved algebraically so inputs map to outputs in a
   * fixed computation order. Compare with InvertedPendulumSystem.mo which
   * uses the implicit (acausal) form where the solver handles the coupling.
   *==========================================================================*/

  import Modelica.Constants.pi;

  PIDControllerBlock controller(
    Kp = 100.0,
    Ki = 20.0,
    Kd = 20.0,
    F_max = 50.0,
    integral_limit = 5.0,
    Ts = 0.005,
    k_swing = 8.0,
    switch_angle = 0.3,
    m_pend = 0.2,
    L_pend = 0.5)
    annotation(Placement(transformation(extent = {{-60, -20}, {-20, 20}})));

  PendulumPlant plant(
    M = 1.0,
    m = 0.2,
    L = 0.5,
    b = 0.1,
    theta_0 = Modelica.Constants.pi)
    annotation(Placement(transformation(extent = {{20, -20}, {60, 20}})));

  /* === Convenience outputs for plotting === */
  Real theta_deg = plant.angle * 180.0 / pi  "Pendulum angle [deg]";
  Real cart_x = plant.x  "Cart position [m]";

equation
  // Controller output -> plant input
  connect(controller.force, plant.force)
    annotation(Line(points = {{-18, 0}, {18, 0}}, color = {0, 0, 127}));

  // Plant outputs -> controller inputs (feedback)
  connect(plant.angle, controller.angle)
    annotation(Line(points = {{62, 12}, {80, 12}, {80, 40}, {-80, 40}, {-80, 8}, {-62, 8}},
                    color = {0, 0, 127}));

  connect(plant.angularRate, controller.angularRate)
    annotation(Line(points = {{62, -12}, {80, -12}, {80, -40}, {-80, -40}, {-80, -8}, {-62, -8}},
                    color = {0, 0, 127}));

  // Position feedback connections
  connect(plant.cartPosition, controller.cartPosition)
    annotation(Line(points = {{62, 20}, {84, 20}, {84, 48}, {-84, 48}, {-84, 16}, {-62, 16}},
                    color = {0, 0, 127}));

  connect(plant.cartVelocity, controller.cartVelocity)
    annotation(Line(points = {{62, -20}, {84, -20}, {84, -48}, {-84, -48}, {-84, -16}, {-62, -16}},
                    color = {0, 0, 127}));

  annotation(
    experiment(
      StartTime = 0,
      StopTime = 20,
      Tolerance = 1e-6,
      Interval = 0.001),
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}})),
    Documentation(info = "<html>
      <h2>Block Diagram View</h2>
      <p>This model presents the inverted pendulum control system as a classical
         signal-flow block diagram. Open it in OMEdit's Diagram View to see the
         controller and plant as separate blocks with feedback connections.</p>
      <p>This is the same system as <code>InvertedPendulumSystem</code>, just
         expressed in causal (block diagram) form rather than acausal (equation) form.</p>
      </html>"));
end SystemBlockDiagram;
