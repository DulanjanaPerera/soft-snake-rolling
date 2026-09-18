# Development Plan — Continuum-First Variable-Curvature Rolling

## 1. Research objective

Develop a general planar rolling-locomotion model for a continuously curved soft robotic snake.

Existing validated model:

\[
(\Phi,\dot\theta)\mapsto u.
\]

New fixed-shape target:

\[
\boxed{
[\kappa(s),\dot\theta(s),p(s),\mu(s)]
\mapsto
\boldsymbol\xi=
[u,v,\omega]^T
}
\]

For the physical three-module robot:

\[
\boxed{
(\phi_1,\phi_2,\phi_3,\dot\theta)
\mapsto
(u,v,\omega).
}
\]

Outputs then determine body-frame ICR, path curvature, turning radius, and global trajectory.

## 2. Why continuum first

The previous rolling paper derives motion from an infinitesimal disk and a continuous friction integral. The new theory should generalize that continuum model directly.

The existing Simscape implementation uses 30 disks per module. It is an independent finite-dimensional physics validation environment after the continuum equations are implemented.

Research hierarchy:

\[
\boxed{
\text{continuum theory}
\rightarrow
\text{quadrature/convergence}
\rightarrow
\text{90-disk Simscape}
\rightarrow
\text{physical robot}
}
\]

The theory must not depend on the number of Simscape disks.

## 3. Scope of Version 1

Included:

- planar centerline;
- flat floor;
- fixed spatial curvature during each instantaneous solve;
- quasi-static locomotion;
- continuous contact;
- constant circular body radius;
- isotropic friction;
- uniform normal-load density;
- synchronized rolling rate across modules.

Excluded initially:

- \(\dot\phi_k\) / shape-change velocity;
- body inertia;
- intermittent contact;
- anisotropic friction;
- dynamic normal-load transfer;
- actuator dynamics;
- inverse navigation control.

## 4. Mathematical development sequence

### Phase A — centerline geometry

Use arc length \(s\in[0,L]\). Let \(s_m=L/2\).

Midpoint body frame:

\[
\beta(s_m)=\pi/2,
\qquad
\mathbf r(s_m)=0.
\]

\[
\beta(s)=\frac{\pi}{2}+\int_{s_m}^{s}\kappa(\sigma)d\sigma,
\]

\[
\mathbf t(s)=
[\cos\beta(s),\sin\beta(s)]^T,
\]

\[
\mathbf d(s)=
\mathbf t(s)\times\hat{\mathbf k}
=
[t_y,-t_x]^T,
\]

\[
\mathbf r(s)=\int_{s_m}^{s}\mathbf t(\sigma)d\sigma.
\]

For constant curvature \(\kappa_0\), define

\[
\alpha=\kappa_0(s-s_m),\qquad \rho=1/\kappa_0.
\]

Recover

\[
\mathbf t=
[-\sin\alpha,\cos\alpha]^T,
\]

\[
\mathbf d=
[\cos\alpha,\sin\alpha]^T,
\]

\[
\mathbf r=
\rho[\cos\alpha-1,\sin\alpha]^T.
\]

This is Gate 1.

### Phase B — local contact slip

Body twist:

\[
\boldsymbol\xi=[u,v,\omega]^T.
\]

Rigid centerline-point velocity:

\[
\mathbf v_c(s)=
[u-\omega y(s),\;v+\omega x(s)]^T.
\]

Fixed-shape contact slip:

\[
\boxed{
\mathbf v_s(s)=
\mathbf v_c(s)
-
R\dot\theta(s)\mathbf d(s).
}
\]

Constant curvature with \(v=\omega=0\) must give

\[
\mathbf v_s(\alpha)=
[u-R\dot\theta\cos\alpha,\;-R\dot\theta\sin\alpha]^T.
\]

This is Gate 2.

### Phase C — distributed friction

Initial model:

\[
\mathbf f(s)=
-\mu p(s)
\frac{\mathbf v_s(s)}
{\sqrt{\mathbf v_s^T\mathbf v_s+\epsilon_v^2}}.
\]

Initial pressure:

\[
p(s)=mg/L.
\]

### Phase D — force and yaw-moment equilibrium

\[
F_x=\int f_x ds,
\qquad
F_y=\int f_y ds,
\]

\[
M_z=\int(xf_y-yf_x)ds.
\]

Solve

\[
\boxed{F_x=F_y=M_z=0}
\]

for

\[
\boxed{u,v,\omega}.
\]

Use scaled solver residuals for conditioning while preserving raw SI-valued outputs.

### Phase E — constant-curvature recovery

Independent old scalar model:

\[
\int_{-\Phi/2}^{\Phi/2}
\frac{\gamma-\cos\alpha}
{\sqrt{1+\gamma^2-2\gamma\cos\alpha}}
d\alpha=0,
\]

\[
u_{old}=\gamma R\dot\theta.
\]

Generalized solver must show:

\[
v\to0,\qquad \omega\to0,\qquad u\to u_{old}.
\]

Sweep \(\Phi\) and compare. This is Gate 4.

### Phase F — piecewise-variable curvature

For the three-module robot:

\[
\kappa(s)=\phi_k/L_k
\]

within module k.

Initially,

\[
\dot\theta_k=\dot\theta.
\]

Study:

1. equal module curvature;
2. one-module perturbation;
3. perturbation sweeps;
4. curvature gradients;
5. ICR/path-curvature maps.

### Phase G — continuum quadrature convergence

Evaluate increasingly dense spatial grids. Prefer odd N so the midpoint is included.

Suggested:

\[
N=31,61,91,181,361,721.
\]

Track

\[
u_N,\quad v_N,\quad \omega_N
\]

and wrench residuals.

This establishes numerical convergence independently of Simscape.

### Phase H — Simscape validation

Existing environment:

- 3 modules;
- 30 disks/module;
- 90 disks total;
- gravity and contact/friction physics.

Run matched shape and rolling-rate cases.

Compare:

- \(u\);
- \(v\);
- \(\omega\);
- path curvature;
- turning radius;
- ICR when well-conditioned.

Where possible compare local contact velocities and normal forces.

### Phase I — physical robot validation

Use motion capture to estimate

\[
x(t),y(t),\psi(t)
\]

and derive body-frame

\[
u(t),v(t),\omega(t).
\]

Use the same curvature cases as theory and Simscape.

Separate error sources: geometry, load distribution, friction variation, tubing, inertia, pneumatic tracking error, and contact loss.

## 5. Planned MATLAB structure

startup.m
- add source/config/test paths;
- keep dependencies explicit.

config/default_parameters.m
- section lengths, radius, mass, friction, rolling rate, solver tolerances, quadrature resolution, regularization.

src/geometry/centerline_from_curvature.m
- output beta, r, tangent, rolling direction;
- midpoint-anchored frame.

src/geometry/piecewise_constant_curvature.m
- construct kappa(s) from section lengths and phi_k.

src/contact/contact_velocity.m
- implement fixed-shape v_s(s).

src/contact/friction_density.m
- regularized isotropic Coulomb law.

src/locomotion/wrench_residual.m
- integrate force and yaw moment.

src/locomotion/solve_body_twist.m
- solve three nonlinear equilibrium equations.

src/locomotion/icr_from_twist.m
- compute ICR with small-omega handling.

src/locomotion/path_curvature_from_twist.m
- path curvature and turn radius.

src/locomotion/solve_gamma_constant_curvature.m
- independent old scalar regression model.

## 6. Planned examples

### demo_centerline_geometry.m
Plot centerline, tangent, rolling direction, curvature.

### demo_constant_curvature_rolling.m
Compare generalized solver to old scalar model.

### demo_variable_curvature_rolling.m
Plot piecewise-curved shape, slip vectors, friction vectors, solved twist, and ICR.

### demo_curvature_sweep.m
Map curvature perturbations to \(u,v,\omega\) and path curvature.

## 7. Mandatory tests

### Geometry
- zero curvature -> straight line;
- constant positive curvature -> analytic circle;
- constant negative curvature -> mirrored circle;
- midpoint frame satisfied.

### Contact velocity
- recover old Eq. (8);
- \(\dot\theta=0\) removes rolling contribution;
- zero twist and zero rolling gives zero slip.

### Wrench
- symmetric constant curvature cancels lateral force/yaw moment at the solution;
- dimensions/signs checked;
- moment-origin change does not alter moment after net force is zero.

### Solver
- constant-curvature twist matches old scalar solution;
- reversing \(\dot\theta\) reverses twist under rate-independent model;
- scaling \(\dot\theta\) scales twist approximately linearly for small regularization;
- reasonable initial guesses converge consistently;
- regularization sensitivity documented.

### Convergence
- increasing spatial resolution converges for both constant and variable curvature.

## 8. Numerical issues

### Coulomb singularity
At zero slip the direction is undefined. Use configurable regularization first. Later consider subgradient/complementarity if needed.

### Residual scaling
Moment and force have different units/scales. Scale moment with a characteristic length in the nonlinear solver.

### Near-zero yaw
ICR is ill-conditioned as \(\omega\to0\). Return a status flag plus Inf/NaN rather than a misleading huge finite ICR.

### Curvature discontinuities
PCC curvature is discontinuous at module boundaries; position and tangent remain continuous.

### Grid placement
The spatial grid should contain midpoint and section boundaries deliberately.

## 9. Definition of first successful prototype

Version 1 succeeds when:

1. arbitrary sampled \(\kappa(s)\) generates a valid centerline;
2. constant-curvature geometry matches the analytic circle;
3. local contact slip matches old Eq. (8);
4. the 3-equation wrench solver reproduces the old scalar speed;
5. unequal module curvatures produce converged \(u,v,\omega\);
6. at least one variable-curvature case is compared with Simscape;
7. assumptions and sign conventions are explicit and tested.

## 10. Later extensions

### Time-varying shape

\[
\mathbf v_{shape}(s)=
\frac{\partial\mathbf r(s,q)}{\partial q}\dot q.
\]

Then

\[
\mathbf v_s=
\mathbf v_{body}+
\mathbf v_{shape}+
\mathbf v_{roll}.
\]

### Nonuniform load/contact
Use \(p(s,q)\), Simscape-estimated pressure, or measured data.

### Dynamics
Add translational/yaw inertia, dynamic normal force, contact loss, and actuation dynamics.

### Steering/control
Build forward map

\[
(\phi_1,\phi_2,\phi_3,\dot\theta)\mapsto(u,v,\omega)
\]

then an approximate inverse for desired path curvature/yaw.

## 11. Immediate coding task

Implement only Phase A:

1. repository skeleton;
2. default parameters;
3. midpoint-centered continuum geometry;
4. piecewise-curvature helper;
5. geometry demo;
6. geometry unit tests.

Do not implement friction or fsolve until geometry tests pass.
