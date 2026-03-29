within InvertedPendulumPID;

function PIDFunction "Modelica wrapper for the external C PID controller"
  /*--------------------------------------------------------------------------
   * This function is the bridge between Modelica and the C firmware code.
   * It maps Modelica variables to the C function's parameters.
   *
   * In a real project, you would have your actual MCU firmware source file
   * (pid_controller.c) and this Modelica function would call into it.
   * OpenModelica compiles the C code and links it automatically.
   *--------------------------------------------------------------------------*/

  /* === Inputs from sensors / simulation === */
  input Real angle "Pendulum angle from vertical [rad], 0 = upright";
  input Real angle_setpoint "Desired angle [rad]";
  input Real angular_rate "Pendulum angular velocity [rad/s]";

  /* === Controller state (passed in from discrete variables) === */
  input Real integral_in "Integral accumulator from previous step [rad*s]";

  /* === Timing === */
  input Real dt "Control loop sample period [s]";

  /* === Tuning parameters === */
  input Real Kp "Proportional gain [N/rad]";
  input Real Ki "Integral gain [N/(rad*s)]";
  input Real Kd "Derivative gain [N*s/rad]";
  input Real force_max "Actuator force saturation [N]";
  input Real integral_max "Anti-windup integral limit [rad*s]";

  /* === Outputs === */
  output Real force_out "Force command to linear motor [N]";
  output Real integral_out "Updated integral accumulator [rad*s]";

  external "C" pid_controller_step(
    angle, angle_setpoint, angular_rate, integral_in,
    dt, Kp, Ki, Kd, force_max, integral_max,
    force_out, integral_out)
    annotation(
      Include = "#include \"pid_controller.c\"",
      IncludeDirectory = "modelica://InvertedPendulumPID/Resources/Include"
    );

  annotation(Documentation(info = "<html>
    <p>Calls the external C function <code>pid_controller_step()</code> which
       implements a discrete PID controller with anti-windup and output saturation.</p>
    <p>The C source is located in <code>Resources/Include/pid_controller.c</code>.</p>
    </html>"));
end PIDFunction;
