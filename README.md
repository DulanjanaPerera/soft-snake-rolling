# soft-snake-rolling

MATLAB research code for **variable-curvature rolling locomotion and steering of a multi-section soft robotic snake**.

## Current development branch

**dev/continuum-rolling-model-v1**

The initial implementation is intentionally **continuum first**:

~~~text
continuum geometry
        ↓
distributed contact slip
        ↓
distributed friction + yaw moment
        ↓
solve body twist [u, v, omega]
        ↓
constant-curvature regression
        ↓
piecewise-variable curvature
        ↓
quadrature convergence
        ↓
90-disk Simscape validation
        ↓
physical robot validation
~~~

The existing Simscape model uses 30 disks per module (90 total for the 3-module snake). It is treated as an independent validation environment, not as the derivation basis.

## Start here

For Codex and contributors:

1. read [AGENTS.md](AGENTS.md);
2. read [docs/DEVELOPMENT_PLAN.md](docs/DEVELOPMENT_PLAN.md);
3. read [docs/MODEL_REFERENCE.md](docs/MODEL_REFERENCE.md);
4. read [docs/VALIDATION_PLAN.md](docs/VALIDATION_PLAN.md).

## Immediate milestone

Implement only the continuum geometry layer first:

\[
\kappa(s)
\rightarrow
\beta(s)
\rightarrow
\mathbf t(s),\mathbf d(s),\mathbf r(s).
\]

The first gate is recovery of the analytic constant-curvature circular arc. Friction and nonlinear body-twist solving should not be implemented until the geometry tests pass.

## Core research target

For the first fixed-shape model:

\[
[\kappa(s),\dot\theta(s),p(s),\mu(s)]
\longrightarrow
[u,v,\omega]^T.
\]

For the 3-module piecewise-constant-curvature robot:

\[
(\phi_1,\phi_2,\phi_3,\dot\theta)
\longrightarrow
(u,v,\omega).
\]

The new model must recover the previously validated constant-curvature rolling-speed result before it is used for variable-curvature predictions.
