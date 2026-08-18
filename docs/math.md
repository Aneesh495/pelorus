# Pelorus: Mathematical Foundations and Derivations

This document details the rigorous theoretical and mathematical derivations underlying the algorithms implemented in the Pelorus spatial perception engine.

---

## 1. Lie Groups and Lie Algebras on SE(3)

Rigid-body spatial transformations in 3D Euclidean space form the Special Euclidean Lie group $\text{SE}(3)$:

$$\text{SE}(3) = \left\{ \mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0}^T & 1 \end{bmatrix} \in \mathbb{R}^{4 \times 4} \;\middle|\; \mathbf{R} \in \text{SO}(3), \; \mathbf{p} \in \mathbb{R}^3 \right\}$$

where the Special Orthogonal group $\text{SO}(3)$ represents rotations:

$$\text{SO}(3) = \{ \mathbf{R} \in \mathbb{R}^{3 \times 3} \mid \mathbf{R}^T \mathbf{R} = \mathbf{I}, \; \det(\mathbf{R}) = +1 \}$$

### 1.1 Tangent Space and the Hat / Vee Operators

The Lie algebra $\mathfrak{so}(3)$ is the tangent space to $\text{SO}(3)$ at the identity, isomorphic to $\mathbb{R}^3$ via the skew-symmetric cross-product operator $(\cdot)^\wedge$:

$$\boldsymbol{\omega} = \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \end{bmatrix} \in \mathbb{R}^3 \iff \boldsymbol{\omega}^\wedge = [\boldsymbol{\omega}]_\times = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix} \in \mathfrak{so}(3)$$

The inverse operator is the vee map: $([\boldsymbol{\omega}]_\times)^\vee = \boldsymbol{\omega}$.

Similarly, the tangent space to $\text{SE}(3)$ is the Lie algebra $\mathfrak{se}(3)$, parameterized by a 6-dimensional twist vector $\boldsymbol{\xi} = [\boldsymbol{\rho}^T, \boldsymbol{\phi}^T]^T \in \mathbb{R}^6$, where $\boldsymbol{\rho} \in \mathbb{R}^3$ corresponds to translational velocity and $\boldsymbol{\phi} \in \mathbb{R}^3$ corresponds to rotational velocity:

$$\boldsymbol{\xi}^\wedge = \begin{bmatrix} [\boldsymbol{\phi}]_\times & \boldsymbol{\rho} \\ \mathbf{0}^T & 0 \end{bmatrix} \in \mathfrak{se}(3)$$

### 1.2 Exponential and Logarithmic Maps on SO(3)

The exponential map $\exp: \mathfrak{so}(3) \to \text{SO}(3)$ maps tangent vectors to finite rotation matrices. For $\boldsymbol{\phi} \in \mathbb{R}^3$ with angle $\theta = \|\boldsymbol{\phi}\|_2$:

$$\exp([\boldsymbol{\phi}]_\times) = \mathbf{I} + \frac{\sin \theta}{\theta} [\boldsymbol{\phi}]_\times + \frac{1 - \cos \theta}{\theta^2} [\boldsymbol{\phi}]_\times^2$$

When $\theta < 10^{-4}$, numerical singularity is avoided using 4th-order Taylor series expansions:

$$\frac{\sin \theta}{\theta} \approx 1 - \frac{\theta^2}{6} + \frac{\theta^4}{120}, \quad \frac{1 - \cos \theta}{\theta^2} \approx \frac{1}{2} - \frac{\theta^2}{24} + \frac{\theta^4}{720}$$

The inverse logarithmic map $\log: \text{SO}(3) \to \mathfrak{so}(3)$ recovers the rotation vector:

$$\theta = \arccos\left(\frac{\text{tr}(\mathbf{R}) - 1}{2}\right), \quad [\boldsymbol{\phi}]_\times = \frac{\theta}{2 \sin \theta} (\mathbf{R} - \mathbf{R}^T)$$

### 1.3 Closed-form SE(3) Exponential and Logarithm

For a twist $\boldsymbol{\xi} = [\boldsymbol{\rho}^T, \boldsymbol{\phi}^T]^T \in \mathbb{R}^6$:

$$\exp(\boldsymbol{\xi}^\wedge) = \begin{bmatrix} \exp([\boldsymbol{\phi}]_\times) & \mathbf{V} \boldsymbol{\rho} \\ \mathbf{0}^T & 1 \end{bmatrix}$$

where the left Jacobian of $\text{SO}(3)$, denoted $\mathbf{V} = \mathbf{J}_l(\boldsymbol{\phi})$, is given analytically by:

$$\mathbf{J}_l(\boldsymbol{\phi}) = \mathbf{I} + \frac{1 - \cos \theta}{\theta^2} [\boldsymbol{\phi}]_\times + \frac{\theta - \sin \theta}{\theta^3} [\boldsymbol{\phi}]_\times^2$$

The inverse Jacobian $\mathbf{J}_l^{-1}(\boldsymbol{\phi})$ allows exact logarithmic inversion on $\text{SE}(3)$:

$$\mathbf{J}_l^{-1}(\boldsymbol{\phi}) = \mathbf{I} - \frac{1}{2} [\boldsymbol{\phi}]_\times + \left( \frac{1}{\theta^2} - \frac{1 + \cos \theta}{2 \theta \sin \theta} \right) [\boldsymbol{\phi}]_\times^2$$

$$\boldsymbol{\phi} = \log_{\text{SO}(3)}(\mathbf{R}), \quad \boldsymbol{\rho} = \mathbf{J}_l^{-1}(\boldsymbol{\phi}) \mathbf{p}$$

### 1.4 Adjoint Representation

The adjoint operator $\text{Ad}_{\mathbf{T}}: \mathfrak{se}(3) \to \mathfrak{se}(3)$ transforms tangent vectors between coordinate frames:

$$\text{Ad}_{\mathbf{T}} = \begin{bmatrix} \mathbf{R} & [\mathbf{p}]_\times \mathbf{R} \\ \mathbf{0} & \mathbf{R} \end{bmatrix} \in \mathbb{R}^{6 \times 6}$$

---

## 2. 15-State Continuous-Time Error-State Kalman Filter (ESKF)

Pelorus tracks the true state $\mathbf{x}$ using nominal state $\hat{\mathbf{x}}$ and error state $\delta \mathbf{x}$:

$$\mathbf{x} = \hat{\mathbf{x}} \oplus \delta \mathbf{x}$$

### 2.1 State Definitions

The 15-dimensional state vector comprises:

$$\mathbf{x} = \begin{bmatrix} \mathbf{p}^T & \mathbf{v}^T & \mathbf{q}^T & \mathbf{b}_a^T & \mathbf{b}_g^T \end{bmatrix}^T$$

- Position: $\mathbf{p} \in \mathbb{R}^3$
- Linear Velocity: $\mathbf{v} \in \mathbb{R}^3$
- Attitude Quaternion: $\mathbf{q} \in \mathbb{H}$
- Accelerometer Bias: $\mathbf{b}_a \in \mathbb{R}^3$
- Gyroscope Bias: $\mathbf{b}_g \in \mathbb{R}^3$

The minimal 15-dimensional error state is:

$$\delta \mathbf{x} = \begin{bmatrix} \delta \mathbf{p}^T & \delta \mathbf{v}^T & \delta \boldsymbol{\theta}^T & \delta \mathbf{b}_a^T & \delta \mathbf{b}_g^T \end{bmatrix}^T \in \mathbb{R}^{15}$$

### 2.2 Continuous Kinematics and Error Dynamics

Given noisy IMU measurements $\tilde{\mathbf{a}} = \mathbf{a} + \mathbf{b}_a + \mathbf{R}^T \mathbf{g} + \mathbf{n}_a$ and $\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega} + \mathbf{b}_g + \mathbf{n}_g$:

$$\dot{\hat{\mathbf{p}}} = \hat{\mathbf{v}}$$
$$\dot{\hat{\mathbf{v}}} = \hat{\mathbf{R}} (\tilde{\mathbf{a}} - \hat{\mathbf{b}}_a) + \mathbf{g}$$
$$\dot{\hat{\mathbf{R}}} = \hat{\mathbf{R}} [\tilde{\boldsymbol{\omega}} - \hat{\mathbf{b}}_g]_\times$$
$$\dot{\hat{\mathbf{b}}}_a = \mathbf{0}, \quad \dot{\hat{\mathbf{b}}}_g = \mathbf{0}$$

Perturbing these equations yields the first-order linearized error dynamics:

$$\delta \dot{\mathbf{x}}(t) = \mathbf{F}_c(t) \delta \mathbf{x}(t) + \mathbf{G}_c(t) \mathbf{w}(t)$$

where the continuous system matrix is:

$$\mathbf{F}_c = \begin{bmatrix} \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & -\hat{\mathbf{R}} [\tilde{\mathbf{a}} - \hat{\mathbf{b}}_a]_\times & -\hat{\mathbf{R}} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & -[\tilde{\boldsymbol{\omega}} - \hat{\mathbf{b}}_g]_\times & \mathbf{0} & -\mathbf{I} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \end{bmatrix}$$

### 2.3 Discrete Covariance Propagation

Over integration interval $\Delta t$, the discrete transition matrix is approximated via Taylor series:

$$\boldsymbol{\Phi} = \exp(\mathbf{F}_c \Delta t) \approx \mathbf{I} + \mathbf{F}_c \Delta t + \frac{1}{2} \mathbf{F}_c^2 \Delta t^2$$

The covariance matrix $\mathbf{P} \in \mathbb{R}^{15 \times 15}$ propagates as:

$$\mathbf{P}_{k|k-1} = \boldsymbol{\Phi}_k \mathbf{P}_{k-1|k-1} \boldsymbol{\Phi}_k^T + \mathbf{Q}_d$$

### 2.4 Numerically Stable Joseph-Form Measurement Update

When a LiDAR scan match provides pose measurement $\mathbf{z} = [\mathbf{p}_m^T, \mathbf{q}_m^T]^T$:

$$\mathbf{y} = \begin{bmatrix} \mathbf{p}_m - \hat{\mathbf{p}} \\ \log_{\text{SO}(3)}(\hat{\mathbf{R}}^T \mathbf{R}_m) \end{bmatrix} \in \mathbb{R}^6$$

$$\mathbf{S} = \mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R}_{\text{meas}}$$
$$\mathbf{K} = \mathbf{P} \mathbf{H}^T \mathbf{S}^{-1}$$
$$\delta \mathbf{x} = \mathbf{K} \mathbf{y}$$

To guarantee positive semi-definiteness and eliminate numerical asymmetry drift, covariance is updated using the Joseph stabilized formulation:

$$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K} \mathbf{H})^T + \mathbf{K} \mathbf{R}_{\text{meas}} \mathbf{K}^T$$

Finally, the error state is injected into the nominal state:

$$\hat{\mathbf{p}} \leftarrow \hat{\mathbf{p}} + \delta \mathbf{p}, \quad \hat{\mathbf{v}} \leftarrow \hat{\mathbf{v}} + \delta \mathbf{v}, \quad \hat{\mathbf{q}} \leftarrow \hat{\mathbf{q}} \otimes \exp\left(\frac{1}{2} \delta \boldsymbol{\theta}\right), \quad \hat{\mathbf{b}} \leftarrow \hat{\mathbf{b}} + \delta \mathbf{b}$$

and the error state is reset: $\delta \mathbf{x} \leftarrow \mathbf{0}$.

---

## 3. Point-to-Plane Iterative Closest Point (ICP)

Point-to-Plane ICP aligns source points $\mathbf{p}_i$ with target surface points $\mathbf{q}_i$ having outward normal unit vector $\mathbf{n}_i$.

### 3.1 Objective Function with Huber Robust M-Estimator

The residual is the signed orthogonal distance from transformed point $\mathbf{T} \mathbf{p}_i$ to the target tangent plane:

$$r_i(\mathbf{T}) = (\mathbf{R} \mathbf{p}_i + \mathbf{t} - \mathbf{q}_i)^T \mathbf{n}_i$$

We minimize the robust sum:

$$\min_{\mathbf{T} \in \text{SE}(3)} \sum_{i=1}^N \rho(r_i(\mathbf{T}))$$

where $\rho(r)$ is the Huber loss function with threshold $\delta_H$:

$$\rho(r) = \begin{cases} \frac{1}{2} r^2 & |r| \le \delta_H \\ \delta_H |r| - \frac{1}{2} \delta_H^2 & |r| > \delta_H \end{cases}$$

The corresponding Iteratively Reweighted Least Squares (IRLS) weights are:

$$w_i = \begin{cases} 1.0 & |r_i| \le \delta_H \\ \frac{\delta_H}{|r_i|} & |r_i| > \delta_H \end{cases}$$

### 3.2 Linearized Gauss-Newton Normal Equations

Linearizing around current estimate $\mathbf{T}$ with incremental twist $\boldsymbol{\xi} = [\boldsymbol{\rho}^T, \boldsymbol{\phi}^T]^T$:

$$\mathbf{R}(\boldsymbol{\phi}) \mathbf{p}_i + \mathbf{t}(\boldsymbol{\rho}) \approx \mathbf{p}_i' + [\boldsymbol{\phi}]_\times \mathbf{p}_i' + \boldsymbol{\rho} = \mathbf{p}_i' - [\mathbf{p}_i']_\times \boldsymbol{\phi} + \boldsymbol{\rho}$$

where $\mathbf{p}_i' = \mathbf{R} \mathbf{p}_i + \mathbf{t}$. The Jacobian row $\mathbf{J}_i \in \mathbb{R}^{1 \times 6}$ is:

$$\mathbf{J}_i = \begin{bmatrix} \mathbf{n}_i^T & (\mathbf{p}_i' \times \mathbf{n}_i)^T \end{bmatrix}$$

Accumulating across all inliers yields the symmetric $6 \times 6$ linear system:

$$\mathbf{H} = \sum_{i=1}^N w_i \mathbf{J}_i^T \mathbf{J}_i, \quad \mathbf{b} = -\sum_{i=1}^N w_i \mathbf{J}_i^T r_i$$

$$\mathbf{H} \Delta \boldsymbol{\xi} = \mathbf{b}$$

Solved via Cholesky $LL^T$ decomposition, updating $\mathbf{T} \leftarrow \exp(\Delta \boldsymbol{\xi}^\wedge) \mathbf{T}$.

---

## 4. Analytical Quintic Minimum-Jerk Spline Trajectories

Smooth kinodynamic trajectories minimize the continuous jerk squared:

$$J = \int_0^T \|\mathbf{j}(t)\|_2^2 \, dt = \int_0^T \left( \dddot{p}_x(t)^2 + \dddot{p}_y(t)^2 + \dddot{p}_z(t)^2 \right) dt$$

By calculus of variations, Euler-Lagrange optimality requires $\frac{d^6 p}{dt^6} = 0$, yielding 5th-order polynomials:

$$p(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5$$

Boundary conditions at $t = 0$:

$$p(0) = p_0 \implies c_0 = p_0$$
$$\dot{p}(0) = v_0 \implies c_1 = v_0$$
$$\ddot{p}(0) = a_0 \implies c_2 = \frac{1}{2} a_0$$

Let $\Delta p = p_T - c_0 - c_1 T - c_2 T^2$, $\Delta v = v_T - c_1 - 2 c_2 T$, and $\Delta a = a_T - 2 c_2$.
The remaining coefficients are solved in closed form:

$$c_3 = \frac{10 \Delta p}{T^3} - \frac{4 \Delta v}{T^2} + \frac{\Delta a}{2 T}$$
$$c_4 = -\frac{15 \Delta p}{T^4} + \frac{7 \Delta v}{T^3} - \frac{\Delta a}{T^2}$$
$$c_5 = \frac{6 \Delta p}{T^5} - \frac{3 \Delta v}{T^4} + \frac{\Delta a}{2 T^3}$$

This analytical formulation eliminates numerical optimization iterations, guaranteeing $C^2$ continuity everywhere with zero runtime matrix factorization.
