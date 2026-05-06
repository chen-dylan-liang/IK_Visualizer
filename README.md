# IK Visualizer

Final project for Yale CPSC 4870/5870 (SP26): 3D Spatial Modeling and Computing.

This project includes Three.js visualizers for robot forward and inverse kinematics. The main inverse kinematics demo is [visualizers/ik.html](visualizers/ik.html).

## Running
The IK solver is live [here](https://chen-dylan-liang.github.io/IK_Visualizer/visualizers/ik.html).

To serve the repository locally, enter the following command in its root directory:
```bash
python3 -m http.server 8000
```
 Then open the IK visualizer in a browser:
```text
http://127.0.0.1:8000/visualizers/ik.html
```

A local server is recommended because the page loads ES modules and GLB mesh assets.

## IK Visualizer

Use the left panel to choose the robot, inspect the solved DOF values, toggle mesh/wireframe display, toggle smooth IK, reset the state, and show or hide link frames. The DOF sliders and numeric fields are passive readouts: the IK solver updates them as targets are dragged.

Interaction:

- Select `XArm7` or `Unitree dog + arm` from the Robot menu.
- Drag a colored IK target sphere in the scene.
- The colored sphere marks the current end-effector position.
- The colored line shows the current position error from the end-effector to its target.
- Enable `Smooth` to penalize large joint changes between frames.

### XArm7 Mode

XArm7 has one draggable target at the TCP. The IK solve uses the linear track and all seven arm joints. Dragging the target updates the whole XArm chain to minimize TCP position error.

### Unitree Dog + Arm Mode

Unitree mode provides five draggable targets: one for each foot and one for the mounted arm end-effector. The trunk is treated as a movable base that can translate but never rotate.

Each target is solved independently:

- A foot target uses trunk translation plus that leg's hip, thigh, and calf joints.
- The arm target uses trunk translation plus the mounted arm joints.
- Other feet and the arm are not included as constraints when one target is active.

Because trunk translation is shared by the kinematic model, dragging one target can translate the whole robot, but only the active target's error appears in the optimization objective.

## Method

For an active end-effector with position $x(q)$ and target $x^*$, the default objective is:

$$
\min_q ||x(q) - x^*||^2
$$

With `Smooth` enabled, the solver adds a frame-to-frame regularization term:

$$
\min_q ||x(q) - x^*||^2 + \lambda ||q - q_{\text{prev}}||^2
$$

where $q_{\text{prev}}$ is the previous frame's joint vector for the active solve variables.

The solver uses a damped Gauss-Newton / Levenberg-Marquardt style step. Let $e = x(q) - x^*$ and $J = \frac{\partial x}{\partial q}$. Each iteration solves:

$$
(J^T J + \mu I + \lambda I)\Delta q =
-J^T e - \lambda(q - q_{\text{prev}})
$$

Then the update is line-searched, joint-limited by clamping, and applied only if it decreases the objective.

The position Jacobian is computed analytically from the current forward kinematics:

- Revolute joint $i$: $J_i = a_i \times (x - o_i)$
- Prismatic joint $i$: $J_i = a_i$
- Movable trunk translation DOFs: treated as prismatic axes

Here $a_i$ is the joint axis in world coordinates and $o_i$ is the joint origin in world coordinates. For Unitree, trunk rotation DOFs are excluded from every IK target group, so the base can translate but not rotate.
