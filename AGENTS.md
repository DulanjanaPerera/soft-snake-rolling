# AGENTS.md — Soft Snake Rolling Development Guide

## Purpose

This repository develops a MATLAB implementation of a **continuum-first model for variable-curvature rolling locomotion of a soft robotic snake**.

Development sequence:

1. continuum geometry;
2. distributed contact slip;
3. distributed friction force and yaw moment;
4. solve planar body twist;
5. recover the previously validated constant-curvature rolling model;
6. extend to piecewise-variable curvature;
7. numerical quadrature/convergence;
8. validate against the existing Simscape model (30 disks per module, 3 modules);
9. validate against the physical soft robot;
10. only afterward add time-varying shape, dynamics, and navigation/control.

The continuum theory is the source of truth. Simscape is an independent physics-based validation environment, not the derivation basis.

## Branch policy

Active development branch: **dev/continuum-rolling-model-v1**

Do not modify or merge to main unless explicitly requested by the user.

Make small, milestone-oriented commits. Do not combine geometry, friction, solver, Simscape integration, and control in one large change.

## Required reading before coding

Read:

- docs/DEVELOPMENT_PLAN.md
- docs/MODEL_REFERENCE.md
- docs/VALIDATION_PLAN.md

Previous paper:

**Rolling Locomotion in Soft Robotic Snakes: Modeling, Control, and Experimental Validation**

The new code must reproduce that paper's constant-curvature rolling result before variable-curvature predictions are trusted.

## Critical notation

Never conflate:

- \(\phi_k\): bending magnitude/subtended angle of module k.
- \(\Phi\): total subtended angle for a uniform-curvature snake.
- \(\theta_k\): bending-plane angle of module k.
- \(\dot\theta\): windmill/rolling actuation rate; **not** body yaw rate.
- \(\omega\): planar body yaw rate.
- \(\kappa(s)\): signed centerline curvature.
- \(u,v\): body-frame translational velocity components.

Use \(\omega\) only for body yaw in the new model.

## Body-frame convention

Use the centerline midpoint \(s_m=L/2\) as body-frame origin.

At the midpoint choose

\[
\mathbf t(s_m)=
\begin{bmatrix}0\\1\end{bmatrix},
\qquad
\mathbf d(s_m)=
\begin{bmatrix}1\\0\end{bmatrix},
\]

where \(\mathbf t\) is the local tangent/rolling axis and

\[
\mathbf d(s)=\mathbf t(s)\times\hat{\mathbf k}
\]

is the local rolling direction in the plane.

Equivalently,

\[
\beta(s_m)=\pi/2.
\]

For constant curvature, with

\[
\alpha=\kappa_0(s-L/2),
\]

this gives

\[
\mathbf t=
\begin{bmatrix}-\sin\alpha\\\cos\alpha\end{bmatrix},
\qquad
\mathbf d=
\begin{bmatrix}\cos\alpha\\\sin\alpha\end{bmatrix},
\]

matching the previous paper's cylindrical basis:
\(\mathbf t=\mathbf e_\alpha\), \(\mathbf d=\mathbf e_r\).

This convention makes the generalized contact-slip equation reduce directly to the old Eq. (8).

## Core fixed-shape continuum model

Initially,

\[
\dot\kappa(s,t)=0.
\]

Geometry:

\[
\frac{d\beta}{ds}=\kappa(s),
\]

with midpoint condition \(\beta(L/2)=\pi/2\),

\[
\mathbf t(s)=
\begin{bmatrix}
\cos\beta(s)\\
\sin\beta(s)
\end{bmatrix},
\]

\[
\frac{d\mathbf r}{ds}=\mathbf t(s),
\qquad
\mathbf r(L/2)=\mathbf 0.
\]

Planar body twist:

\[
\boldsymbol\xi=
\begin{bmatrix}
u\\v\\\omega
\end{bmatrix}.
\]

Centerline point velocity from body motion:

\[
\mathbf v_c(s)=
\begin{bmatrix}
u\\v
\end{bmatrix}
+
\omega\hat{\mathbf k}\times\mathbf r(s)
=
\begin{bmatrix}
u-\omega y(s)\\
v+\omega x(s)
\end{bmatrix}.
\]

Local rolling direction:

\[
\mathbf d(s)=
\mathbf t(s)\times\hat{\mathbf k}
=
\begin{bmatrix}
t_y(s)\\
-t_x(s)
\end{bmatrix}.
\]

For local rolling rate \(\dot\theta(s)\) and body radius \(R\),

\[
\boxed{
\mathbf v_s(s)
=
\begin{bmatrix}
u\\v
\end{bmatrix}
+
\omega\hat{\mathbf k}\times\mathbf r(s)
-
R\dot\theta(s)\mathbf d(s)
}
\]

for the adopted sign convention.

Constant-curvature check:

\[
\omega=0,\quad v=0
\]

must give

\[
\mathbf v_s(\alpha)=
\begin{bmatrix}
u-R\dot\theta\cos\alpha\\
-R\dot\theta\sin\alpha
\end{bmatrix},
\]

which is the previous paper's Eq. (8).

Initial regularized isotropic Coulomb friction density:

\[
\mathbf f(s)=
-\mu(s)p(s)
\frac{\mathbf v_s(s)}
{\sqrt{\mathbf v_s^T(s)\mathbf v_s(s)+\epsilon_v^2}}.
\]

Quasi-static distributed wrench balance:

\[
F_x=\int f_x(s)\,ds=0,
\]

\[
F_y=\int f_y(s)\,ds=0,
\]

\[
M_z=
\int
\left[x(s)f_y(s)-y(s)f_x(s)\right]ds=0.
\]

Solve for \(u,v,\omega\).

For numerical conditioning, a scaled solver residual may be used:

\[
\mathbf R_s=
\begin{bmatrix}
F_x/F_0\\
F_y/F_0\\
M_z/(F_0L)
\end{bmatrix},
\]

while preserving raw physical residuals.

## Initial assumptions

Keep explicit:

1. flat horizontal ground;
2. inextensible centerline;
3. planar centerline geometry;
4. circular cross-section with constant radius R;
5. restricted/no backbone twist in the rolling model;
6. continuous ground contact over the modeled interval;
7. fixed spatial curvature during each instantaneous locomotion solve;
8. quasi-static body motion;
9. isotropic Coulomb friction initially;
10. uniform normal-load density initially, \(p(s)=mg/L\);
11. same rolling rate in all three modules initially;
12. local rolling axis is the centerline tangent;
13. no shape-rate velocity term until the fixed-shape model is validated.

Do not silently relax assumptions.

## MATLAB architecture

Preferred structure:

~~~text
config/
src/geometry/
src/contact/
src/locomotion/
src/utilities/
examples/
tests/
simscape/
docs/
~~~

Core planned functions:

~~~text
centerline_from_curvature.m
piecewise_constant_curvature.m
contact_velocity.m
friction_density.m
wrench_residual.m
solve_body_twist.m
icr_from_twist.m
path_curvature_from_twist.m
solve_gamma_constant_curvature.m
~~~

Functions should be small, deterministic, and traceable to one mathematical relation where practical.

Use structs for model/parameter data rather than long positional argument lists as the API grows.

## Numerical conventions

- SI units only.
- Model angles are radians.
- Use explicit names such as theta_dot and omega_body where ambiguity is possible.
- Do not hardcode physical parameters in computational functions.
- Put defaults in config/default_parameters.m.
- Regularization \(\epsilon_v\) must be configurable and sensitivity-tested.
- Integration resolution must remain independent of the 90 physical Simscape disks.
- A 90-point quadrature and a 90-disk Simscape model are not the same physical model.

## Development gates

Do not proceed past a gate until tests pass.

### Gate 1 — Geometry
Constant \(\kappa\) reproduces the analytic circular arc and local tangent/rolling-direction fields.

### Gate 2 — Contact kinematics
Constant curvature with \(v=\omega=0\) reproduces the old Eq. (8) pointwise.

### Gate 3 — Wrench implementation
Numerical force/moment integration passes symmetry and origin-consistency checks.

### Gate 4 — Constant-curvature regression
The general solver reproduces the previous scalar model:
\(v\approx0\), \(\omega\approx0\), \(u\approx\gamma(\Phi)R\dot\theta\).

### Gate 5 — Variable curvature
Only after Gate 4, allow \(\phi_1\ne\phi_2\ne\phi_3\) and study \(u,v,\omega\).

### Gate 6 — Simscape
Only after quadrature convergence, compare to the 30-disks/module Simscape model.

### Gate 7 — Hardware
Only after simulation discrepancies are understood, compare to motion-capture experiments.

## Constant-curvature regression model

For uniform curvature, the old normalized speed coefficient \(\gamma\) satisfies

\[
\int_{-\Phi/2}^{\Phi/2}
\frac{\gamma-\cos\alpha}
{\sqrt{1+\gamma^2-2\gamma\cos\alpha}}
\,d\alpha=0,
\]

with

\[
u=\gamma R\dot\theta.
\]

Implement this independently as solve_gamma_constant_curvature.m to serve as a regression oracle.

Do not tune the new model to variable-curvature data before it reproduces this benchmark.

## Three-module first variable-curvature case

For section lengths \(L_k\),

\[
\kappa_k=\frac{\phi_k}{L_k}
\]

within module k.

Start with synchronized rolling:

\[
\dot\theta_1=\dot\theta_2=\dot\theta_3=\dot\theta.
\]

Initial studies:

1. equal module curvature;
2. perturb one module by \(\Delta\phi\);
3. sweep \(\Delta\phi\);
4. curvature gradients;
5. compare \(u,v,\omega\), path curvature, and turning radius.

Do not assume a yaw-sign result for a mirrored perturbation until the adopted frame/sign convention is explicitly verified.

## ICR and path outputs

For \(|\omega|>\omega_{tol}\),

\[
\mathbf r_{ICR}=
\begin{bmatrix}
-v/\omega\\
u/\omega
\end{bmatrix}.
\]

\[
\kappa_{path}=
\frac{\omega}{\sqrt{u^2+v^2}}.
\]

\[
R_{turn}=
\frac{\sqrt{u^2+v^2}}{|\omega|}.
\]

Treat the ICR as an output of the locomotion solution, not as the constraint that determines motion.

## Simscape role

Existing Simscape model: 30 disks/module, 90 total for three modules.

Hierarchy:

\[
\text{continuum theory}
\rightarrow
\text{numerical quadrature}
\rightarrow
\text{Simscape multibody model}
\rightarrow
\text{physical robot}.
\]

Where available, compare local contact velocity, contact forces, and normal loads in addition to final body velocity.

Refinement sequence:

1. uniform \(p(s)\);
2. Simscape-estimated normal-load distribution;
3. investigate remaining inertial/contact-compliance effects.

## Later work — do not implement prematurely

After fixed-shape quasi-static validation:

1. add \(\dot\kappa\ne0\);
2. add shape velocity \(\partial\mathbf r/\partial q\,\dot q\);
3. allow section-specific \(\dot\theta_k\);
4. add anisotropic friction only if needed;
5. model nonuniform pressure/contact loss;
6. add full dynamics/inertia;
7. develop inverse steering/navigation control.

## Expectations for Codex

For each milestone:

1. state which mathematical equation is implemented;
2. keep changes limited to that milestone;
3. add/update automated tests;
4. run tests;
5. report numerical tolerances and failed assumptions;
6. do not change model equations merely to make a test pass;
7. document frame/sign ambiguity instead of guessing;
8. preserve constant-curvature regression;
9. do not modify Simscape files unless explicitly requested;
10. do not merge this development branch into main without explicit user approval.
