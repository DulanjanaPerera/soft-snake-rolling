# Model Reference and Sign Conventions

This is the mathematical reference for Version 1. If code behavior disagrees with this file, investigate the frame/sign convention rather than silently changing equations.

## 1. Existing constant-curvature benchmark

Using old angular coordinate \(\alpha\),

\[
\mathbf e_r=
\begin{bmatrix}
\cos\alpha\\
\sin\alpha
\end{bmatrix},
\qquad
\mathbf e_\alpha=
\begin{bmatrix}
-\sin\alpha\\
\cos\alpha
\end{bmatrix}.
\]

Local angular velocity:

\[
\dot\theta\,\mathbf e_\alpha.
\]

With scalar body translation \(u\hat{\mathbf i}\), ground-contact slip:

\[
\boxed{
\mathbf v_s(\alpha)=
\begin{bmatrix}
u-R\dot\theta\cos\alpha\\
-R\dot\theta\sin\alpha
\end{bmatrix}.
}
\]

Normalized speed:

\[
u=\gamma R\dot\theta,
\]

where

\[
\boxed{
\int_{-\Phi/2}^{\Phi/2}
\frac{\gamma-\cos\alpha}
{\sqrt{1+\gamma^2-2\gamma\cos\alpha}}
\,d\alpha=0.
}
\]

This is the regression benchmark.

## 2. New body frame

Let

\[
s_m=L/2.
\]

Centerline midpoint is the origin:

\[
\mathbf r(s_m)=0.
\]

Orient the frame so

\[
\mathbf t(s_m)=
\begin{bmatrix}
0\\1
\end{bmatrix},
\qquad
\beta(s_m)=\pi/2.
\]

Define rolling direction

\[
\boxed{
\mathbf d(s)=\mathbf t(s)\times\hat{\mathbf k}
}
\]

or in 2D,

\[
\mathbf d=
\begin{bmatrix}
t_y\\
-t_x
\end{bmatrix}.
\]

At the midpoint,

\[
\mathbf d(s_m)=
\begin{bmatrix}
1\\0
\end{bmatrix}.
\]

Thus body +x is the nominal rolling direction, matching the constant-curvature paper.

## 3. Curvature to centerline

\[
\boxed{
\kappa(s)=\frac{d\beta}{ds}.
}
\]

\[
\boxed{
\beta(s)=
\frac{\pi}{2}
+
\int_{s_m}^{s}\kappa(\sigma)d\sigma.
}
\]

\[
\boxed{
\mathbf t(s)=
\begin{bmatrix}
\cos\beta(s)\\
\sin\beta(s)
\end{bmatrix}.
}
\]

\[
\boxed{
\mathbf r(s)=
\int_{s_m}^{s}\mathbf t(\sigma)d\sigma.
}
\]

## 4. Constant-curvature reduction

Let \(\kappa=\kappa_0\),

\[
\rho=1/\kappa_0,
\qquad
\alpha=\kappa_0(s-s_m).
\]

Then

\[
\beta=\pi/2+\alpha,
\]

\[
\mathbf t=
\begin{bmatrix}
-\sin\alpha\\
\cos\alpha
\end{bmatrix}
=
\mathbf e_\alpha,
\]

\[
\mathbf d=
\begin{bmatrix}
\cos\alpha\\
\sin\alpha
\end{bmatrix}
=
\mathbf e_r,
\]

and

\[
\boxed{
\mathbf r=
\rho
\begin{bmatrix}
\cos\alpha-1\\
\sin\alpha
\end{bmatrix}.
}
\]

## 5. Body twist

\[
\boxed{
\boldsymbol\xi=
\begin{bmatrix}
u\\v\\\omega
\end{bmatrix}.
}
\]

- \(u\): body +x translational velocity.
- \(v\): body +y translational velocity.
- \(\omega\): body yaw rate about +z.

Important:

\[
\omega\neq\dot\theta.
\]

Centerline-point body-motion velocity:

\[
\boxed{
\mathbf v_c(s)=
\begin{bmatrix}
u-\omega y(s)\\
v+\omega x(s)
\end{bmatrix}.
}
\]

## 6. Rolling contribution

Local cross-sectional angular velocity:

\[
\boldsymbol\omega_r(s)=
\dot\theta(s)\mathbf t(s).
\]

Centerline-to-floor contact vector:

\[
\boldsymbol\rho_c=-R\hat{\mathbf k}.
\]

Therefore

\[
\boldsymbol\omega_r\times\boldsymbol\rho_c
=
-R\dot\theta\mathbf d(s).
\]

Fixed-shape slip:

\[
\boxed{
\mathbf v_s(s)=
\begin{bmatrix}
u-\omega y(s)\\
v+\omega x(s)
\end{bmatrix}
-
R\dot\theta(s)\mathbf d(s).
}
\]

For constant curvature with \(v=0,\omega=0\), this becomes exactly the old contact-slip expression.

## 7. Friction density

Initial law:

\[
\boxed{
\mathbf f(s)=
-\mu(s)p(s)
\frac{\mathbf v_s(s)}
{\sqrt{\|\mathbf v_s(s)\|^2+\epsilon_v^2}}.
}
\]

Units:

- \(p\): N/m;
- \(\mathbf f\): N/m;
- \(\epsilon_v\): m/s.

Initial uniform pressure:

\[
p(s)=mg/L.
\]

Regularization is numerical; it is not a complete static-friction model.

## 8. Wrench

\[
\mathbf F=
\int_0^L\mathbf f(s)ds.
\]

\[
\boxed{
M_z=
\int_0^L
[x(s)f_y(s)-y(s)f_x(s)]ds.
}
\]

Quasi-static locomotion:

\[
\boxed{
F_x=0,\qquad F_y=0,\qquad M_z=0.
}
\]

These determine \(u,v,\omega\).

## 9. Post-processing

For sufficiently nonzero \(\omega\),

\[
\boxed{
\mathbf r_{ICR}=
\begin{bmatrix}
-v/\omega\\
u/\omega
\end{bmatrix}.
}
\]

\[
\boxed{
\kappa_{path}=
\frac{\omega}{\sqrt{u^2+v^2}}.
}
\]

\[
\boxed{
R_{turn}=
\frac{\sqrt{u^2+v^2}}{|\omega|}.
}
\]

## 10. Three-module PCC specialization

For lengths \(L_1,L_2,L_3\) and bending angles \(\phi_1,\phi_2,\phi_3\),

\[
\kappa_k=\phi_k/L_k.
\]

Use piecewise constant \(\kappa(s)\) in the same continuum equations.

Initially use

\[
\dot\theta_1=
\dot\theta_2=
\dot\theta_3.
\]

## 11. Later time-varying-shape term

Do not include in Version 1.

When \(q(t)\) changes,

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

Derive carefully to avoid double-counting bending-plane rolling.
