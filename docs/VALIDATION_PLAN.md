# Validation Plan

## Validation philosophy

Every layer must be validated against a simpler layer:

\[
\text{analytic special cases}
\rightarrow
\text{continuum numerical model}
\rightarrow
\text{quadrature convergence}
\rightarrow
\text{Simscape}
\rightarrow
\text{physical robot}.
\]

## V0 — numerical sanity

Check:

- no NaN/Inf in ordinary configurations;
- SI units;
- midpoint included in spatial grid;
- section boundaries represented deliberately;
- vector dimensions consistent;
- solver diagnostics returned.

## V1 — geometry

### Straight line

Input:

\[
\kappa(s)=0.
\]

Expected:

- \(\beta(s)=\pi/2\);
- \(x(s)=0\);
- \(y(s)=s-L/2\);
- tangent constant;
- rolling direction +x.

### Constant curvature

Compare numerical output to

\[
\mathbf r=
\rho
[\cos\alpha-1,\sin\alpha]^T.
\]

Record maximum position/tangent errors versus grid resolution.

### Signed curvature

Repeat with negative curvature and verify the corresponding mirror geometry.

## V2 — local contact kinematics

For constant curvature and

\[
\boldsymbol\xi=[u,0,0]^T,
\]

compare numerical slip pointwise with

\[
[u-R\dot\theta\cos\alpha,\;-R\dot\theta\sin\alpha]^T.
\]

Also test:

- \(\dot\theta=0\);
- \(u=v=\omega=\dot\theta=0\);
- nonzero pure yaw against direct rigid-body velocity.

## V3 — constant-curvature locomotion

Implement an independent scalar solver for \(\gamma(\Phi)\).

For a sweep of \(\Phi\):

1. solve old scalar model;
2. solve new 3-DOF wrench balance;
3. compare \(u\);
4. report \(v\);
5. report \(\omega\);
6. report raw wrench residual.

Suggested plots:

- \(\gamma\) vs \(\Phi\);
- relative error in \(u\);
- \(|v|\) vs \(\Phi\);
- \(|\omega|\) vs \(\Phi\).

Do not proceed to variable curvature if disagreement is unexplained.

## V4 — solver invariances/sensitivity

### Rate reversal

Under fixed-shape rate-independent Coulomb friction,

\[
\dot\theta\rightarrow-\dot\theta
\]

should reverse solved twist within numerical/regularization tolerance.

### Rate scaling

For \(c>0\),

\[
\dot\theta\rightarrow c\dot\theta
\]

should approximately produce

\[
\boldsymbol\xi\rightarrow c\boldsymbol\xi
\]

when regularization is sufficiently small.

### Initial guess

Solve from multiple reasonable initial guesses and check consistency.

### Regularization

Sweep \(\epsilon_v\) and document convergence/stability.

## V5 — variable curvature

Use the three-module PCC model.

Cases:

1. equal \(\phi_k\);
2. perturb module 1;
3. perturb module 2;
4. perturb module 3;
5. curvature gradients.

Record:

- \(u,v,\omega\);
- path curvature;
- turning radius;
- ICR status;
- force/moment residual;
- slip/friction distributions.

Do not impose a steering-sign expectation until frame/mirror symmetry is analytically verified.

## V6 — quadrature convergence

For representative constant- and variable-curvature cases, increase N and compare to a high-resolution reference.

Track relative changes in

\[
u,v,\omega.
\]

A 90-point quadrature is useful for comparison but has no privileged status merely because Simscape has 90 physical disks.

## V7 — Simscape validation

Existing model:

- 30 disks/module;
- 3 modules;
- 90 disks total.

Use matched:

- section lengths;
- radius;
- mass;
- \(\phi_k\);
- \(\dot\theta_k\);
- equivalent friction parameters where possible.

Compare body twist and trajectory outputs.

If available, export per-disk:

- center position/orientation;
- angular velocity;
- contact normal force;
- tangential friction force;
- contact point velocity.

### Refinement order if theory disagrees with Simscape

1. verify geometry/frame/signs;
2. verify contact kinematics;
3. use Simscape normal-load distribution in continuum friction integral;
4. examine contact loss;
5. examine inertia;
6. examine contact compliance;
7. only then consider changing friction law.

Do not jump directly to empirical fitting.

## V8 — hardware validation

Use motion capture to estimate global pose.

Transform derivatives into body-frame \(u,v\), and estimate yaw rate \(\omega\).

Repeat the same shape inputs used in theory and Simscape.

Track shape-control error so model error is not confused with failure to realize desired \(\phi_k\).

## Acceptance criteria

Precise tolerances should be established from numerical convergence and measurement uncertainty, not invented in advance.

Every test must:

- define its tolerance explicitly;
- justify it;
- avoid passing because of an overly loose threshold.
