within InvertedPendulumPID;

model InvertedPendulum3D "3D animated inverted pendulum with swing-up and PID balance"
  /*==========================================================================
       * MultiBody version of the inverted pendulum system.
       *
       * Uses Modelica.Mechanics.MultiBody components for:
       *   - Physically correct dynamics (joints, bodies, gravity)
       *   - Automatic 3D animation in OMEdit
       *   - Easy attachment of custom STL meshes for realistic visuals
       *
       * The controller uses energy-based swing-up (Astrom & Furuta) to pump
       * the pendulum from hanging down to upright, then switches to PID
       * with position feedback for balancing.
       *==========================================================================*/
  import MB = Modelica.Mechanics.MultiBody;
  import Modelica.Constants.pi;
  /* ============================= Parameters ============================== */
  // Physical plant
  parameter Real M = 1.0 "Cart mass [kg]";
  parameter Real m = 0.2 "Pendulum bob mass [kg]";
  parameter Real L = 0.5 "Pendulum length to center of mass [m]";
  parameter Real b_fric = 0.1 "Cart friction coefficient [N*s/m]";
  // PID tuning (balance mode)
  parameter Real Kp = 100.0 "Proportional gain [N/rad]";
  parameter Real Ki = 20.0 "Integral gain [N/(rad*s)]";
  parameter Real Kd = 20.0 "Derivative gain [N*s/rad]";
  parameter Real F_max = 50.0 "Actuator force saturation [N]";
  parameter Real integral_limit = 5.0 "Anti-windup integral limit [rad*s]";
  // Position feedback (outer loop, active in balance mode)
  parameter Real Kp_pos = 0.3     "Position proportional gain [rad/m]";
  parameter Real Kd_pos = 0.5     "Position derivative gain [rad*s/m]";
  parameter Real x_ref = 0.0      "Desired cart position [m]";
  parameter Real setpoint_max = 0.1  "Max angle setpoint from position loop [rad] (~5.7 deg)";
  // Swing-up parameters
  parameter Real k_swing = 8.0    "Swing-up gain [N/J]";
  parameter Real switch_angle = 0.3  "Angle threshold to switch to PID [rad] (~17 deg)";
  // Timing
  parameter Real Ts = 0.005 "Controller sample period [s] (200 Hz)";
  // Initial conditions
  parameter Real theta_0 = Modelica.Constants.pi "Initial pendulum angle [rad] (pi = hanging down)";
  /* ======================== World and Ground ============================== */
  inner MB.World world(n = {0, -1, 0}, animateWorld = true, animateGravity = false) "Gravity pointing downward (-y)";
  MB.Parts.Fixed ground "Fixed reference frame at origin";
  /* ======================== Track Visualization =========================== */
  MB.Visualizers.FixedShape trackShape(shapeType = "box", length = 4.0, width = 0.1, height = 0.02, lengthDirection = {1, 0, 0}, widthDirection = {0, 0, 1}, r_shape = {-2.0, -0.12, -0.05}, color = {150, 150, 150}) "Visual: horizontal rail the cart slides on";
  /* ======================== Cart ========================================== */
  MB.Joints.Prismatic cartJoint(n = {1, 0, 0}, useAxisFlange = true, s(start = 0, fixed = true), v(start = 0, fixed = true)) "Cart slides along x-axis";
  Modelica.Mechanics.Translational.Sources.Force motorForce "Linear motor: applies horizontal force to cart";
  Modelica.Mechanics.Translational.Components.Damper cartFriction(d = b_fric) "Viscous friction opposing cart motion";
  MB.Parts.Body cartMass(m = M, r_CM = {0, 0, 0}, animation = false) "Cart point mass (visualization handled separately)";
  MB.Visualizers.FixedShape cartShape(shapeType = "box", length = 0.4, width = 0.2, height = 0.1, lengthDirection = {1, 0, 0}, widthDirection = {0, 0, 1}, r_shape = {-0.2, -0.05, -0.1}, color = {80, 80, 220}) "Visual: cart body as a blue box";
  /*
       * --- STL REPLACEMENT ---
       * To use your own cart mesh, replace the cartShape above with:
       *
       *   MB.Visualizers.FixedShape cartShape(
       *     shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/cart.stl",
       *     length = 0.4,
       *     width = 0.2,
       *     height = 0.1,
       *     r_shape = {-0.2, -0.05, -0.1},
       *     color = {80, 80, 220});
       *
       * The STL file must be placed in InvertedPendulumPID/Resources/Meshes/.
       * See 3D_Visualization_Guide.md for full details.
       */
  /* ======================== Pendulum ====================================== */
  MB.Joints.Revolute pendulumJoint(n = {0, 0, -1}, phi(start = theta_0, fixed = true), w(start = 0, fixed = true)) "Pendulum pivot: rotation in the x-y plane.
         n={0,0,-1} so positive phi = tip tilts toward +x (matches sign convention of equation-based model)";
  MB.Parts.FixedTranslation pendulumRod(r = {0, L, 0}, width = 0.02, color = {220, 80, 80}) "Visual: thin rod from pivot to bob (massless rigid link)";
  /*
       * --- STL REPLACEMENT ---
       * To use a custom pendulum rod mesh, replace pendulumRod with:
       *
       *   MB.Parts.FixedTranslation pendulumRod(
       *     r = {0, L, 0},
       *     shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/rod.stl",
       *     width = 0.02);
       */
  MB.Parts.Body pendulumBob(m = m, r_CM = {0, 0, 0}, sphereDiameter = 0.08, sphereColor = {220, 50, 50}, cylinderDiameter = 0) "Pendulum tip mass (shown as red sphere)";
  /*
       * --- STL REPLACEMENT ---
       * To use a custom bob mesh, set:
       *   sphereDiameter = 0
       * and add a FixedShape connected to pendulumBob.frame_a:
       *
       *   MB.Visualizers.FixedShape bobShape(
       *     shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/bob.stl",
       *     length = 0.08, width = 0.08, height = 0.08,
       *     color = {220, 50, 50});
       */
  /* ======================== Discrete Controller =========================== */
  discrete Real F(start = 0, fixed = true) "Motor force command [N]";
  discrete Real integral_state(start = 0, fixed = true) "PID integral accumulator";
  discrete Real ctrl_mode(start = 0, fixed = true) "Controller mode: 0 = swing-up, 1 = balance";
  /* ======================== Convenience Outputs ============================ */
  Real theta_deg = pendulumJoint.phi*180.0/pi "Pendulum angle [deg]";
equation
/* ======================= Mechanical Connections ========================= */
// Ground to cart
  connect(ground.frame_b, cartJoint.frame_a);
// Cart joint frame -> cart mass + cart visual + pendulum pivot
// (multiple things connect to the same frame: they share position/orientation)
  connect(cartJoint.frame_b, cartMass.frame_a);
  connect(cartJoint.frame_b, cartShape.frame_a);
  connect(cartJoint.frame_b, pendulumJoint.frame_a);
// Track visualization on ground
  connect(ground.frame_b, trackShape.frame_a);
// Pendulum chain: pivot -> rod -> bob
  connect(pendulumJoint.frame_b, pendulumRod.frame_a);
  connect(pendulumRod.frame_b, pendulumBob.frame_a);
/* ======================= Force and Friction ============================= */
// Motor force drives the cart axis
  connect(motorForce.flange, cartJoint.axis);
// Friction damper between cart axis and its support (ground reference)
  connect(cartFriction.flange_a, cartJoint.axis);
  connect(cartFriction.flange_b, cartJoint.support);
// Feed PID force output to motor
  motorForce.f = F;
/* ======================= Discrete Controller (swing-up + PID in C) ====== */
  when sample(0, Ts) then
    (F, integral_state, ctrl_mode) = SwingUpFunction(
      pendulumJoint.phi,         // measured angle [rad]
      pendulumJoint.w,           // angular velocity [rad/s]
      pre(cartJoint.s),          // cart position [m]
      pre(cartJoint.v),          // cart velocity [m/s]
      pre(integral_state),       // previous integral state
      pre(ctrl_mode),            // previous mode
      Ts,                        // sample period
      k_swing,                   // swing-up gain
      m, L, world.g,             // plant parameters for energy calc
      Kp, Ki, Kd,               // PID gains
      F_max,                     // actuator saturation
      integral_limit,            // anti-windup limit
      Kp_pos, Kd_pos,           // position feedback gains
      x_ref, setpoint_max,      // position reference
      switch_angle               // switching threshold
    );
  end when;
  annotation(
    experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.001),
    Documentation(info = "<html>
      <h2>3D Animated Inverted Pendulum with Swing-Up</h2>
      <p>This model uses <code>Modelica.Mechanics.MultiBody</code> components to produce
         a 3D animation of the inverted pendulum system. The pendulum starts hanging
         down and an energy-based swing-up controller rocks the cart to pump the
         pendulum up, then switches to PID balancing with position feedback.</p>
      <h3>Viewing the Animation</h3>
      <ol>
        <li>Add <code>+d=visxml</code> in <b>Simulation Setup &gt; Translation Flags</b></li>
        <li>Check <b>Launch Animation</b> in <b>Simulation Setup &gt; General</b></li>
        <li>Simulate the model — the animation appears in the result tab</li>
        <li>Press Play to watch the pendulum swing up and balance</li>
      </ol>
      <h3>Custom STL Meshes</h3>
      <p>See inline comments in the source code and <code>3D_Visualization_Guide.md</code>
         for instructions on replacing the default shapes with your own STL files.</p>
      </html>"),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian +d=visxml",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end InvertedPendulum3D;
