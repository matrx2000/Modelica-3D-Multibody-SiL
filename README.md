# Integrating MCU C Code into OpenModelica: Inverted Pendulum Example

## What This Project Is

This project demonstrates how to take **real C code** — the kind that runs on a
microcontroller (STM32, Arduino, ESP32, etc.) — and plug it into a **Modelica
physics simulation** so you can test your embedded firmware against a realistic
physical plant *before* building hardware.

The concrete example: an **energy-based swing-up controller + PID balancer
written in C** that swings an **inverted pendulum on a cart** from hanging
down to upright and keeps it balanced.

```
                    O  <-- pendulum tip (mass m)
                   /
                  / L
                 /
                / theta
               /
  ============X============     cart (mass M)
  |  |  |  |  |  |  |  |  |    on horizontal track
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            ---> F  (linear motor force)
            ---> x  (cart position)
```

The C code is the controller firmware. Modelica is the physics. OpenModelica
compiles them together and runs a full closed-loop simulation.

---

<iframe width="560" height="315" src="https://www.youtube.com/embed/H58nGgHDc4A?si=_4BMtN7xw4W2pCAg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

---

## Why Modelica? (Acausal vs. Causal Modeling)

If you come from an embedded / controls background, you are probably used to
**causal** simulation tools — Simulink, Python/scipy, hand-written C simulations
— where you explicitly compute outputs from inputs in a fixed order:

```
# Causal (Simulink-style): YOU decide the computation order
error = setpoint - angle
force = Kp * error
acceleration = (force - friction) / mass    # you solve for acceleration
velocity = velocity + acceleration * dt     # you integrate manually
position = position + velocity * dt
```

Modelica works differently. It is an **acausal** (equation-based) language.
You write the physics as **equations**, not assignments, and the solver figures
out what to compute from what:

```modelica
// Acausal (Modelica): you state the PHYSICS, the solver does the math
(M + m) * a + m * L * alpha * cos(theta) - m * L * omega^2 * sin(theta) + b * v = F;
m * L * a * cos(theta) + m * L^2 * alpha - m * g * L * sin(theta) = 0;
```

Notice: these are **simultaneous equations**, not assignments. There is no
`a = ...` on the left side. The two accelerations (`a` and `alpha`) are
**coupled** — you can't solve one without the other. Modelica's DAE
(Differential-Algebraic Equation) solver handles this automatically.

### What this means in practice

| Aspect | Causal (Simulink, C, Python) | Acausal (Modelica) |
|---|---|---|
| **You write** | Step-by-step computation | Physical equations |
| **Computation order** | You decide | Solver decides |
| **Coupled physics** | You must manually rearrange and solve | Just write the equations |
| **Changing what's input vs. output** | Rewrite the code | Change nothing, solver adapts |
| **Physical units/consistency** | Your responsibility | Tool can check |
| **Connecting components** | Wire signal flows | Connect physical ports |

### Why this matters for the pendulum

The inverted pendulum equations are **coupled** — the cart acceleration appears
in the pendulum equation and vice versa. In a causal tool, you would need to
algebraically rearrange these into explicit form (solve the 2x2 system by hand
or in matrix form). In Modelica you just write the two equations as they come
from the physics textbook, and the compiler does the symbolic manipulation.

This is a simple example with only 2 coupled equations. In a real system
(thermal + mechanical + electrical + hydraulic), you might have hundreds of
coupled equations across domains. Rearranging all of those by hand would be
impractical — that's where acausal modeling really shines.

### Where causal code still belongs

The PID controller is **causal** — it reads sensors, computes, writes actuators.
That's exactly what an MCU does. You don't want the solver deciding how your
firmware works; you want deterministic step-by-step execution.

This project shows the natural split:
- **Physics (plant):** acausal, in Modelica — because physics is equations
- **Controller (firmware):** causal, in C — because firmware is algorithms

---

## Project Structure — What Goes Where

```
InvertedPendulumPID/
│
├── package.mo                        [1] Package definition
├── package.order                         Component ordering
│
├── PIDFunction.mo                    [2] The BRIDGE: Modelica <-> C (PID only)
├── SwingUpFunction.mo                    The BRIDGE: Modelica <-> C (swing-up + PID)
│
├── InvertedPendulumSystem.mo         [3] Physics + controller wiring (equation-based)
│
├── InvertedPendulum3D.mo            [7] 3D animated model (MultiBody)
│
├── PendulumPlant.mo                  [8] Causal plant block (for block diagram)
├── PIDControllerBlock.mo             [9] Swing-up + PID block (for block diagram)
├── SystemBlockDiagram.mo            [10] Block diagram: controller + plant in feedback
│
├── Resources/
│   ├── Include/
│   │   ├── pid_controller.h          [4] C header — PID only (MCU firmware API)
│   │   ├── pid_controller.c          [5] C implementation — PID only
│   │   ├── swing_up_controller.h         C header — swing-up + PID state machine
│   │   └── swing_up_controller.c         C implementation — energy-based swing-up + PID
│   └── Meshes/                           (place custom STL files here)
│
└── simulate.mos                      [6] Command-line simulation script
```

### [1] `package.mo` — The Package Definition

Standard Modelica boilerplate. Declares `InvertedPendulumPID` as a loadable
package so OpenModelica can find all the sub-components.

### [2] `PIDFunction.mo` — The Bridge Between Worlds

This is the key file that makes C integration work. It is a Modelica `function`
that declares inputs and outputs in Modelica types, then maps them to a C
function call:

```modelica
function PIDFunction
  input Real angle;
  input Real angle_setpoint;
  // ... more inputs ...
  output Real force_out;
  output Real integral_out;

  external "C" pid_controller_step(
    angle, angle_setpoint, ...,
    force_out, integral_out)
    annotation(
      Include = "#include \"pid_controller.c\"",
      IncludeDirectory = "modelica://InvertedPendulumPID/Resources/Include"
    );
end PIDFunction;
```

**How it works:**
- `external "C"` tells the Modelica compiler: "don't look for a Modelica body,
  call this C function instead"
- `Include` tells it which C source file to compile
- `IncludeDirectory` tells it where to find that file (using the Modelica URI
  scheme `modelica://PackageName/path`)
- OpenModelica's code generator compiles the C file with its built-in C compiler
  (MinGW on Windows, gcc on Linux) and links it into the simulation executable
- At runtime, every call to `PIDFunction(...)` in Modelica becomes a direct call
  to `pid_controller_step(...)` in the compiled C code

**Output parameters:** Notice that `force_out` and `integral_out` are Modelica
`output` variables. In the C function they are `double *` (pointers). The
Modelica compiler handles this mapping automatically — Modelica outputs become
C pointer parameters.

### [3] `InvertedPendulumSystem.mo` — The Main Model

This file contains everything needed to simulate the full system:

**Physical parameters** (lines 32-36):
```modelica
parameter Real M = 1.0   "Cart mass [kg]";
parameter Real m = 0.2   "Pendulum bob mass [kg]";
parameter Real L = 0.5   "Pendulum length [m]";
parameter Real b = 0.1   "Cart friction [N*s/m]";
```

**Continuous plant dynamics** (lines 91-98) — the acausal physics equations:
```modelica
der(x) = v;
der(theta) = omega;
der(v) = a;
der(omega) = alpha;

(M + m) * a + m * L * alpha * cos(theta) - m * L * omega^2 * sin(theta) + b * v = F;
m * L * a * cos(theta) + m * L^2 * alpha - m * g * L * sin(theta) = 0;
```

**Discrete controller** (lines 110-121) — calls the C code at fixed intervals:
```modelica
when sample(0, Ts) then
  (F, integral_state) = PIDFunction(
    theta, 0.0, omega, pre(integral_state),
    Ts, Kp, Ki, Kd, F_max, integral_limit
  );
end when;
```

The `when sample(0, Ts)` block fires every `Ts` seconds (5 ms = 200 Hz). This
is exactly how a real MCU timer interrupt works — the controller "wakes up" at
a fixed rate, reads sensors, computes, and writes to the actuator. Between
samples, `F` holds its last value (zero-order hold), just like a real DAC output.

The `pre(integral_state)` operator gets the value from the *previous* discrete
step — this is how Modelica handles discrete state that persists between
samples, analogous to a `static` variable in a C ISR.

### [4]-[5] `pid_controller.h/.c` — The MCU Firmware

This is standard C code, identical to what you would compile for an STM32
or Arduino. There is nothing Modelica-specific in it:

- **Proportional term:** Immediate response to angle error
- **Integral term:** Eliminates steady-state offset, with anti-windup clamping
  to prevent accumulator overflow (critical on real hardware)
- **Derivative term:** Uses angular rate directly (derivative-on-measurement)
  instead of differencing the error — avoids derivative kick on setpoint changes
- **Output saturation:** Clamps force to `[-force_max, +force_max]`, mimicking
  the real motor's physical limit
- **Back-calculation anti-windup:** When output saturates, the integration step
  is undone to prevent the integral from winding up further

On a real MCU, the output would go to a PWM register or DAC. Here it goes back
to Modelica as a force value.

### [6] `simulate.mos` — Command-Line Runner

An OpenModelica script (`.mos`) that loads and simulates the model from the
terminal without needing the GUI. Useful for batch runs and CI/CD.

### [7] `InvertedPendulum3D.mo` — 3D Animated Model

This is the same inverted pendulum system, rebuilt using the
`Modelica.Mechanics.MultiBody` library. Instead of hand-written equations, the
physics come from MultiBody joints and bodies:

- **`Prismatic` joint** — cart slides along the x-axis
- **`Revolute` joint** — pendulum rotates at the pivot
- **`Body` / `FixedTranslation`** — masses and rigid links
- **`Visualizers.FixedShape`** — 3D shapes (box for cart, cylinder for rod,
  sphere for bob) attached to each body

The discrete PID controller is identical — the same C code called via
`PIDFunction` at 200 Hz.

After simulating in OMEdit, open **View > Windows > Animation** to watch the
cart slide and the pendulum swing in 3D. You can orbit, zoom, and pan the view.

**Custom STL meshes:** You can replace the default shapes with your own CAD
models (exported as STL). Place them in `Resources/Meshes/` and change the
`shapeType` parameter from `"box"` to a `modelica://` URI pointing to your file.
See `3D_Visualization_Guide.md` for full instructions.

### [8]-[10] Block Diagram Models

These three files present the same control system as a **signal-flow block
diagram** — the way you'd draw it on a whiteboard or see it in a controls
textbook:

```
  setpoint ──┐
             ▼
         ┌────────┐       force       ┌────────┐
         │  PID   │ ───────────────> │  Plant  │
         │ (C)    │                   │         │
         │        │ <─────────────── │         │
         └────────┘   angle, omega    └────────┘
```

- **`PendulumPlant.mo`** [8] — The plant dynamics in **explicit causal form**.
  The coupled equations of motion are algebraically solved into `a = f(...)` and
  `alpha = f(...)` so that inputs (force) map to outputs (angle, rate) in a fixed
  computation order. This is the same math as [3], just rearranged so it works as
  a block with defined inputs and outputs.

- **`PIDControllerBlock.mo`** [9] — A Modelica `block` that wraps the C PID
  function. Takes angle and angular rate as inputs, outputs motor force. Fires at
  fixed sample rate just like [3].

- **`SystemBlockDiagram.mo`** [10] — Wires the plant and controller blocks in a
  feedback loop. Open this in OMEdit's **Diagram View** to see the blocks and
  connections visually. Simulates identically to the equation-based model.

---

## How to Run

### In OMEdit (GUI)

1. Open OMEdit
2. **File > Open Model/Library File** > select `InvertedPendulumPID/package.mo`
3. In the Libraries Browser, expand `InvertedPendulumPID`

**Equation-based model (2D plots):**

4. Double-click **InvertedPendulumSystem**
5. Click the **Simulate** button (green play)
6. In the Variables Browser, check these to plot:
   - `theta_deg` — pendulum angle in degrees (should converge to 0)
   - `F` — motor force command (staircase from discrete sampling)
   - `x` — cart position (moves to catch the pendulum)
   - `pendulum_tip_x`, `pendulum_tip_y` — pendulum tip trajectory

**3D animated model:**

4. Double-click **InvertedPendulum3D**
5. Open **Simulation Setup** (wrench icon or **Simulation > Simulation Setup**)
6. Under the **Translation Flags** tab, add `+d=visxml` in the
   **Additional Translation Flags** field — this tells OpenModelica to generate
   the `_visual.xml` file that the animation viewer requires
7. Under the **General** tab, check **Launch Animation** at the bottom — this
   automatically opens the animation window after simulation completes
8. Check all three **Save ... inside model** checkboxes at the bottom of the
   dialog — this persists the flags into the `.mo` file so you don't have to
   re-enter them each time
9. Click **OK**, then **Simulate**
10. After simulation completes, the animation appears in the result tab. Use the
    **Play** button and **Time** slider in the playback controls to watch the
    cart and pendulum move in 3D
11. Use the mouse to orbit, zoom, and pan the view

> **Troubleshooting:** If the animation view is empty, check that
> `InvertedPendulum3D_visual.xml` exists next to the `_res.mat` file in your
> working directory (**Tools > Options > General > Working Directory**). If it's
> missing, the `+d=visxml` flag was not set.

**Block diagram model:**

4. Double-click **SystemBlockDiagram**
5. Switch to **Diagram View** to see the controller ↔ plant feedback wiring
6. Click **Simulate** — results are identical to the equation-based model

### From Command Line

```bash
cd "path/to/<workfolder>"
omc InvertedPendulumPID/simulate.mos
```

This produces a CSV file with all simulation results.

---

## What You Should See

The pendulum starts **hanging straight down** (`theta_0 = π`). The simulation
shows three phases:

1. **Swing-up (0 - ~5 s):** The energy-based controller rocks the cart back and
   forth, pumping energy into the pendulum. Each swing gets larger until the
   pendulum approaches the upright position. You can watch the `ctrl_mode`
   variable — it stays at 0 during this phase.

2. **Catch and stabilize (~5 - 8 s):** When the pendulum enters the switch zone
   (`|angle| < 0.3 rad`), the controller switches to PID mode (`ctrl_mode = 1`).
   The PID catches the pendulum and damps out oscillations.

3. **Balance (8 - 20 s):** The pendulum is balanced upright. The position
   feedback loop gently drives the cart back toward center.

If you zoom into the `F` signal, you'll see the **staircase pattern** from
discrete sampling — the force updates every 5 ms and is held constant between
updates, exactly like a real digital controller.

> **Tip:** Plot `ctrl_mode` to see exactly when the controller switches from
> swing-up (0) to balance (1). If the pendulum gets knocked away, it will
> switch back to swing-up automatically.

---

## Experiments to Try

| What to change | Parameter | Effect |
|---|---|---|
| Start upright (skip swing-up) | `theta_0 = 0.15` (~8.6 deg) | Goes straight to PID balance mode |
| Slower swing-up | `k_swing = 1.0` | Gentler rocking, takes longer to reach top |
| Faster swing-up | `k_swing = 5.0` | More aggressive rocking, larger cart excursions |
| Earlier PID catch | `switch_angle = 0.5` (~29 deg) | Switches to PID further from upright — riskier |
| Later PID catch | `switch_angle = 0.15` (~8.6 deg) | Switches very close to upright — more reliable |
| Slower controller | `Ts = 0.02` (50 Hz) | Performance degrades, shows sampling effects |
| Weaker motor | `F_max = 20` | Actuator saturates, swing-up takes longer or fails |
| Less damping | `Kd = 5` | More oscillation, slower settling |
| No integral action | `Ki = 0` | May have steady-state angle offset |
| Heavier pendulum | `m = 0.5` | Harder to control, needs re-tuning |
| Longer stick | `L = 1.0` | Slower dynamics, easier to balance |
| Disable position feedback | `Kp_pos = 0, Kd_pos = 0` | Cart drifts freely, only angle is controlled |
| Tighter position hold | `Kp_pos = 1.0, Kd_pos = 1.0` | Cart returns faster but may oscillate more |
| Different cart home | `x_ref = 0.5` | Cart holds at 0.5 m instead of origin |

---

## Known Limitations

The controller handles swing-up from any initial angle, but has practical limits:

| Limitation | What happens | Why |
|---|---|---|
| **Very weak motor** (`F_max` < ~15 N) | Swing-up fails or takes very long | Not enough force to pump sufficient energy into the pendulum |
| **Very slow sampling** (`Ts` > ~0.05 s) | Oscillations grow until instability | Digital control needs sufficient bandwidth relative to plant dynamics |
| **Cart leaves track** | Simulation may diverge | During swing-up the cart moves significantly; if `F_max` is too high or `k_swing` too aggressive the cart can overshoot the track |
| **Noisy switching** | Pendulum bounces between modes | If `switch_angle` is too large, the PID catches the pendulum too far from upright and loses it, triggering swing-up again |

### How the swing-up controller works

The swing-up uses the **Astrom & Furuta energy method**:

1. Compute the pendulum's total energy relative to the upright equilibrium
2. Apply a force proportional to the energy deficit, in the direction that adds
   energy: `F = k_swing * E * omega * cos(theta)`
3. This naturally swings the pendulum in larger and larger arcs
4. When `|angle| < switch_angle`, switch to PID balance mode
5. If the pendulum falls back beyond `1.5 * switch_angle`, revert to swing-up

The `ctrl_mode` variable (0 = swing-up, 1 = balance) can be plotted to see
exactly when transitions occur.

---

## Custom 3D Visuals with STL Meshes

The `InvertedPendulum3D` model uses default shapes (box, cylinder, sphere). You
can replace these with your own CAD-designed STL meshes for a realistic look:

1. **Design** your parts in any CAD tool (FreeCAD, Fusion 360, Blender, etc.)
2. **Export as STL** — use meters as the unit, place the attachment point at the
   origin
3. **Drop files** into `InvertedPendulumPID/Resources/Meshes/`
4. **Edit `InvertedPendulum3D.mo`** — change `shapeType` from `"box"` to:
   ```modelica
   shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/cart.stl"
   ```
5. Adjust `r_shape`, `length`, `width`, `height` to align the mesh visually

The inline comments in `InvertedPendulum3D.mo` mark exactly which lines to
change for the cart, rod, and bob. See **`3D_Visualization_Guide.md`** for the
full walkthrough including coordinate conventions, scaling, and troubleshooting.

---

## The Big Picture: When to Use This Approach

This C-in-Modelica pattern is valuable when:

- You want to **test your real firmware** against a physics model before having
  hardware (Model-in-the-Loop / MIL testing)
- You want to **tune controller gains** without risking a physical prototype
- You need to **validate timing** — does your 5 ms control loop keep up with
  the plant dynamics? What happens at 20 ms?
- You are doing **co-simulation** — the physics team models the plant in
  Modelica, the embedded team writes the controller in C, and both come together
  in one simulation

The same C file (`pid_controller.c`) that runs in this simulation can be
compiled for your target MCU with zero changes. That is the point — one source
of truth for the controller logic, tested in simulation, deployed on hardware.

---

## Requirements

- **OpenModelica 1.26.3 (64-bit)** — https://openmodelica.org/
- **Modelica Standard Library 4.1.0** (ships with OpenModelica)
- A C compiler is included with OpenModelica (MinGW on Windows, gcc on Linux)
- No additional libraries needed
