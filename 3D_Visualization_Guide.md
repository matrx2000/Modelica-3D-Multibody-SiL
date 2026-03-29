# Connecting Custom STL Meshes to OpenModelica 3D Bodies

## Short Answer: Yes, OpenModelica Supports This

OpenModelica's `Modelica.Mechanics.MultiBody` library has built-in 3D animation.
When you simulate a MultiBody model in OMEdit, you can watch the animation in
real time. Every joint, body, and shape in the model gets rendered as a 3D object.

You can replace the default shapes (boxes, cylinders, spheres) with **custom STL
meshes** — your own CAD models of the cart, pendulum rod, bob, etc.

---

## How 3D Animation Works in OpenModelica

### The Pipeline

```
Modelica model (joints, bodies)
        |
        v
Simulation produces .mat result file
  (contains position + orientation of every frame, at every timestep)
        |
        v
OMEdit Animation View reads the result file
  and renders 3D shapes attached to each frame
```

Every `MultiBody` component has one or more **frames** (`frame_a`, `frame_b`).
A frame is a position + orientation in 3D space. During simulation, the solver
computes how each frame moves over time. The animation viewer reads these
trajectories and moves the attached 3D shapes accordingly.

### Viewing the Animation

1. Open **Simulation Setup** for `InvertedPendulum3D` (wrench icon)
2. Under the **Translation Flags** tab, add `+d=visxml` in the
   **Additional Translation Flags** field — this generates the `_visual.xml`
   file that the animation viewer requires
3. Under the **General** tab, check **Launch Animation** at the bottom to
   automatically open the animation window after simulation
4. Check all three **Save ... inside model** checkboxes at the bottom of the
   dialog — this persists the flags into the `.mo` file so you don't have to
   re-enter them each time
5. Click **OK**, then **Simulate**
6. After simulation completes, the animation appears in the result tab. Use the
   **Play** button and **Time** slider in the playback controls to watch the
   cart sliding and the pendulum swinging
7. Use mouse to orbit, zoom, and pan the 3D view

---

## How Shapes Attach to Bodies

In MultiBody, visualization shapes are defined by these parameters on components
like `Body`, `BodyBox`, `BodyCylinder`, `FixedTranslation`, or standalone
`Visualizers.FixedShape`:

| Parameter | What it does |
|---|---|
| `shapeType` | Shape kind: `"box"`, `"cylinder"`, `"sphere"`, `"cone"`, `"pipe"`, `"beam"`, `"gearwheel"`, or **a file path to an STL** |
| `length` | Bounding size in the length direction |
| `width` | Bounding size in the width direction |
| `height` | Bounding size in the height direction |
| `lengthDirection` | Unit vector for the length axis (default `{1,0,0}`) |
| `widthDirection` | Unit vector for the width axis (default `{0,1,0}`) |
| `r_shape` | Offset from the component's frame to the shape origin |
| `color` | RGB color as `{R, G, B}` (0–255 each) |
| `extra` | Shape-specific extra parameter (e.g., inner radius for pipe) |

The shape moves rigidly with the frame it's attached to. If the frame moves
(because a joint is actuated), the shape moves with it.

---

## Using STL Files: Step by Step

### 1. Prepare Your STL Files

**Format requirements:**
- Binary or ASCII STL (binary is smaller/faster)
- Units: **meters** — OpenModelica interprets STL vertex coordinates in meters.
  If your CAD tool exports in millimeters, either scale in the CAD tool before
  exporting, or use the `length`/`width`/`height` parameters to scale in Modelica
- Coordinate system: the STL's local origin (0,0,0) becomes the attachment point.
  Design your mesh so that the natural attachment point is at the origin

**Recommendations:**
- Keep triangle count reasonable (under 50k triangles per part) for smooth animation
- Ensure the mesh is watertight (closed surface) for correct rendering
- Name files descriptively: `cart.stl`, `pendulum_rod.stl`, `pendulum_bob.stl`

### 2. Place Files in the Package

Create a `Meshes` directory inside `Resources`:

```
InvertedPendulumPID/
├── Resources/
│   ├── Include/
│   │   ├── pid_controller.h
│   │   └── pid_controller.c
│   └── Meshes/              <-- NEW: put STL files here
│       ├── cart.stl
│       ├── pendulum_rod.stl
│       └── pendulum_bob.stl
├── InvertedPendulum3D.mo
└── ...
```

### 3. Reference STL Files in Modelica Code

Use the `modelica://` URI scheme to reference files relative to the package:

```modelica
shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/cart.stl"
```

This URI resolves to the file path relative to the `package.mo` location.
It works regardless of where the user has the project on disk.

### 4. Attach to Components

There are three ways to attach an STL shape to a MultiBody component:

#### Option A: Standalone `Visualizers.FixedShape` (most flexible)

```modelica
import MB = Modelica.Mechanics.MultiBody;

// A shape that follows a specific frame
MB.Visualizers.FixedShape cartShape(
  shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/cart.stl",
  // length, width, height SCALE the STL bounding box to these dimensions:
  length = 0.4,         // scale STL to 0.4m in length direction
  width  = 0.2,         // scale STL to 0.2m in width direction
  height = 0.1,         // scale STL to 0.1m in height direction
  lengthDirection = {1, 0, 0},
  widthDirection  = {0, 0, 1},
  // Offset: shift the shape so its visual center aligns with the frame
  r_shape = {-0.2, -0.05, -0.1},
  color = {80, 80, 220}
);

// Connect the shape to the frame you want it to follow
connect(cartJoint.frame_b, cartShape.frame_a);
```

**When to use:** When you want full control over placement, or when attaching
a shape to a component that doesn't have built-in shape parameters (like a
`Body` with `animation = false`).

#### Option B: Directly on `FixedTranslation` (for rods/links)

```modelica
MB.Parts.FixedTranslation pendulumRod(
  r = {0, 0.5, 0},
  shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/pendulum_rod.stl",
  width = 0.02,
  color = {220, 80, 80}
);
```

**When to use:** For structural links between joints. The shape stretches
between `frame_a` and `frame_b` by default.

#### Option C: On a `Body` via sphere/cylinder parameters (limited)

`Body` components show a sphere at the center of mass and a cylinder from
`frame_a` to the CM. You can't directly set `shapeType` on a `Body`. Instead:
- Set `animation = false` on the Body
- Add a separate `Visualizers.FixedShape` connected to the same frame (Option A)

---

## Concrete Example: Replacing Default Shapes in InvertedPendulum3D

The `InvertedPendulum3D.mo` model uses three visual elements:

| Visual | Default shape | Component | Frame |
|---|---|---|---|
| Cart | Blue box | `cartShape` (FixedShape) | `cartJoint.frame_b` |
| Rod | Thin cylinder | `pendulumRod` (FixedTranslation) | `pendulumJoint.frame_b` → `pendulumBob.frame_a` |
| Bob | Red sphere | `pendulumBob` (Body, sphereDiameter) | `pendulumRod.frame_b` |

### Replacing the Cart

In `InvertedPendulum3D.mo`, find the `cartShape` declaration and change `shapeType`:

```modelica
// BEFORE (default box):
MB.Visualizers.FixedShape cartShape(
  shapeType = "box",
  ...);

// AFTER (your STL):
MB.Visualizers.FixedShape cartShape(
  shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/cart.stl",
  length = 0.4,       // target size in meters — STL is scaled to fit
  width  = 0.2,
  height = 0.1,
  r_shape = {-0.2, -0.05, -0.1},   // adjust so the pivot point is at the top center
  color = {80, 80, 220});
```

**STL origin tip:** Design `cart.stl` so that the pendulum pivot point is at
coordinate (0, 0, 0) in the STL file. Then `r_shape` offsets the visual so the
cart body is centered below the pivot. If your pivot is at the top-center of the
cart mesh, set `r_shape = {0, 0, 0}`.

### Replacing the Pendulum Rod

```modelica
// BEFORE:
MB.Parts.FixedTranslation pendulumRod(
  r = {0, L, 0},
  width = 0.02,
  color = {220, 80, 80});

// AFTER:
MB.Parts.FixedTranslation pendulumRod(
  r = {0, L, 0},
  shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/pendulum_rod.stl",
  width = 0.02,
  color = {220, 80, 80});
```

**STL origin tip:** Design `pendulum_rod.stl` with the pivot end at (0,0,0) and
the bob end at (0, L, 0) where L is the pendulum length in meters. The shape
will be scaled to fit between `frame_a` and `frame_b`.

### Replacing the Bob

The bob is a `Body` component that shows a sphere. To replace it with an STL:

```modelica
// 1. Disable the default sphere:
MB.Parts.Body pendulumBob(
  m = m,
  r_CM = {0, 0, 0},
  sphereDiameter = 0,        // <-- hide default sphere
  cylinderDiameter = 0);

// 2. Add a FixedShape for the custom mesh:
MB.Visualizers.FixedShape bobShape(
  shapeType = "modelica://InvertedPendulumPID/Resources/Meshes/pendulum_bob.stl",
  length = 0.08,
  width  = 0.08,
  height = 0.08,
  r_shape = {-0.04, -0.04, -0.04},  // center the mesh on the frame
  color = {220, 50, 50});

// 3. Connect to the same frame:
connect(pendulumRod.frame_b, bobShape.frame_a);
```

---

## How Scaling Works with STL Files

When you set `length`, `width`, `height` on a shape that uses an STL file,
OpenModelica **scales the STL bounding box** to fit those dimensions:

```
STL bounding box (in STL coordinates):
  x: [x_min, x_max]  →  scaled to fit 'length' along lengthDirection
  y: [y_min, y_max]  →  scaled to fit 'width' along widthDirection
  z: [z_min, z_max]  →  scaled to fit 'height' along remaining direction
```

**If your STL is already in meters at the correct scale**, set `length`, `width`,
`height` to match the actual STL bounding box dimensions. Then no scaling occurs.

**If your STL is in millimeters**, you have two options:
1. Set `length`/`width`/`height` to the desired size in meters (auto-scaled)
2. Scale the STL in your CAD tool before exporting

---

## Coordinate System Conventions

```
OpenModelica MultiBody default (matches InvertedPendulum3D.mo):

         +y (up)
          |
          |
          |_______ +x (cart motion direction)
         /
        /
       +z (out of screen / rotation axis)

  Gravity: world.n = {0, -1, 0}  →  gravity in -y direction
  Cart:    slides along +x       →  cartJoint.n = {1, 0, 0}
  Pivot:   rotates around z      →  pendulumJoint.n = {0, 0, -1}
```

**When designing STL files**, use the same coordinate convention:
- Cart: long axis along x, sits below the pivot (y < 0 relative to pivot)
- Rod: extends from (0,0,0) upward along +y to (0, L, 0)
- Bob: centered on (0,0,0)

---

## Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| STL not visible | Wrong file path in `shapeType` | Check the `modelica://` URI matches the actual file location |
| Shape appears but wrong size | STL not in meters | Set `length`/`width`/`height` to desired meter dimensions |
| Shape at wrong position | STL origin doesn't match frame | Adjust `r_shape` to offset, or redesign STL with correct origin |
| Shape rotated wrong | STL axes don't match Modelica axes | Adjust `lengthDirection`/`widthDirection`, or rotate STL in CAD |
| Animation view is empty | `_visual.xml` not generated | Add `+d=visxml` in **Simulation Setup > Translation Flags > Additional Translation Flags** and re-simulate |
| Animation view is empty | Result file not loaded | Check **Launch Animation** in **Simulation Setup > General**, or verify the `.mat` file exists in the working directory (**Tools > Options > General > Working Directory**) |
| "File not found" error | Meshes directory doesn't exist | Create `Resources/Meshes/` and place STL files there |
| Simulation slow with STL | Too many triangles | Decimate the mesh in your CAD tool (aim for < 50k triangles) |

---

## Quick Reference: Modelica URI for Package Resources

The `modelica://` URI scheme resolves paths relative to the package root:

```
modelica://InvertedPendulumPID/Resources/Meshes/cart.stl
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
           Package name          Path from package.mo directory
```

This means if `package.mo` is at:
```
C:\Users\you\project\InvertedPendulumPID\package.mo
```

Then the URI resolves to:
```
C:\Users\you\project\InvertedPendulumPID\Resources\Meshes\cart.stl
```

---

## Summary: What to Do

1. **Model your parts in CAD** (FreeCAD, Fusion 360, SolidWorks, Blender, etc.)
2. **Export as STL** in meters, with the attachment point at the origin
3. **Place in** `InvertedPendulumPID/Resources/Meshes/`
4. **Edit** `InvertedPendulum3D.mo` — change `shapeType` from `"box"`/`"cylinder"` to the `modelica://` URI
5. **Adjust** `r_shape`, `length`, `width`, `height` until the visual aligns
6. **Simulate** and open the Animation view in OMEdit
