within;
package InvertedPendulumPID "Inverted Pendulum on Cart stabilized by a C-based PID Controller"
  annotation(
    version = "1.0.0",
    Documentation(info = "<html>
      <h2>Inverted Pendulum with External C PID Controller</h2>
      <p>This package demonstrates how to integrate C code (representing MCU firmware)
         into a Modelica physics simulation via the <code>external \"C\"</code> interface.</p>
      <h3>Physical System</h3>
      <p>A rigid pendulum (stick) is mounted on a cart that rides on a horizontal track.
         A linear motor applies a horizontal force to the cart. The goal is to balance
         the pendulum in the inverted (upright) position.</p>
      <h3>Controller</h3>
      <p>A discrete-time PID controller implemented in C, called at fixed intervals
         to mimic a real microcontroller's timer interrupt. The C code includes
         anti-windup, derivative-on-measurement, and output saturation.</p>
      </html>"));
end InvertedPendulumPID;
