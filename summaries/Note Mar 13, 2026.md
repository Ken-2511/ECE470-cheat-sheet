Here is a well-structured Markdown summary of the provided lecture notes. 

***Note from the Teaching Assistant:*** *I have reviewed the mathematical derivations in the notes. The logic is largely sound, but there are a few places where the notes stop short (like the kinetic energy of the RR robot). I have filled in these missing gaps to ensure the notes are complete and self-contained.*

---

# Robotics: Holonomic Constraints & Euler-Lagrange Equations

## 1. Core Concepts & Definitions

### Holonomic Constraints
For a system of $N$ particles in 3D space, their positions are given by $r^1, \dots, r^N \in \mathbb{R}^3$. 
A **holonomic constraint** is a restriction on the system's configuration that can be expressed as an algebraic equation:
$$ g(r^1, \dots, r^N) = 0 $$
where $g: \mathbb{R}^{3N} \to \mathbb{R}^\ell$. The constraint surface (configuration space) is the manifold $\mathcal{C} = g^{-1}(0)$.

### Degrees of Freedom & Generalized Coordinates
*   **Degrees of Freedom (DoFs):** $n = 3N - \ell$
*   **Generalized Coordinates:** A set of independent variables $(q_1, \dots, q_n)$ that uniquely parameterize the constraint surface $\mathcal{C}$.

### The Lagrange-D'Alembert Principle
*   **Virtual displacements** are treated as tangent vectors to the constraint manifold $\mathcal{C}$.
*   The principle states that the constraint forces do no virtual work. Therefore, the equations of motion (EOMs) can be found by projecting the Newton-Euler equations onto the tangent space $T_r\mathcal{C}$ using virtual displacements.

### Euler-Lagrange (EL) Equations
The Lagrangian $\mathcal{L}$ is the difference between Kinetic Energy ($K$) and Potential Energy ($P$):
$$ \mathcal{L}(q, \dot{q}) = K(q, \dot{q}) - P(q) $$

The **Euler-Lagrange equations** govern the dynamics of the system:
$$ \frac{d}{dt} \nabla_{\dot{q}} \mathcal{L} - \nabla_q \mathcal{L} = \tau $$
where $\tau$ is the generalized force/torque vector, defined as:
$$ \tau := \left[ \frac{\partial r}{\partial q} \right]^T f_a $$
*(Note: $f_a$ represents the applied external forces).*

---

## 2. Example 1: Simple Pendulum

**System:** A point mass $m$ at the end of a massless rod of length $\ell$.
*(Note on convention: Based on the parameterization below, $q=0$ represents the pendulum hanging straight down along the negative y-axis. We will use the shorthand $s_q = \sin q$ and $c_q = \cos q$).*

**1) Generalized Coordinate:** $n=1$, $q = \theta$ 
**2) Parameterization of $r$:**
$$ r(q) = \ell \begin{bmatrix} s_q \\ -c_q \\ 0 \end{bmatrix} \implies \dot{r} = \ell \begin{bmatrix} c_q \\ s_q \\ 0 \end{bmatrix} \dot{q} $$
**3) Kinetic Energy ($K$):**
$$ K(q, \dot{q}) = \frac{1}{2} m \|\dot{r}\|^2 = \frac{1}{2} m \left\| \ell \begin{bmatrix} c_q \\ s_q \\ 0 \end{bmatrix} \dot{q} \right\|^2 = \frac{1}{2} m \ell^2 \dot{q}^2 $$
**4) Potential Energy ($P$):**
$$ U(r) = mg r_y \implies P(q) = U(r(q)) = mg(-\ell c_q) = -mg\ell c_q $$
**5) Lagrangian ($\mathcal{L}$):**
$$ \mathcal{L} = K - P = \frac{1}{2} m \ell^2 \dot{q}^2 + mg\ell c_q $$
**6) EL Equation:**
$$ \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}} \right) - \frac{\partial \mathcal{L}}{\partial q} = \tau $$
$$ \frac{d}{dt} [m\ell^2 \dot{q}] - (-mg\ell s_q) = \tau $$
$$ m\ell^2 \ddot{q} + mg\ell s_q = \tau \implies m\ell^2 \ddot{q} = -mg\ell s_q + \tau $$
**Generalized Torque:**
$$ \tau = \left(\frac{\partial r}{\partial q}\right)^T f_a = \ell \begin{bmatrix} c_q & s_q & 0 \end{bmatrix} f_a = \text{torque} $$

---

## 3. Example 2: Roller Coaster

**System:** A particle of mass $m$ sliding on a 2D curve defined by $y = f(x)$.

**1) Coordinate and Constraint:** $n=1$. Constraint $g(r) = r_y - f(r_x) = 0$. Let $q = x$.
**2) Parameterization:**
$$ r(q) = \begin{bmatrix} q \\ f(q) \\ 0 \end{bmatrix} $$
**3) Kinetic Energy:**
$$ \dot{r} = \begin{bmatrix} \dot{q} \\ f'(q)\dot{q} \\ 0 \end{bmatrix} = \begin{bmatrix} 1 \\ f'(q) \\ 0 \end{bmatrix} \dot{q} $$
$$ K(q, \dot{q}) = \frac{1}{2} m \|\dot{r}\|^2 = \frac{1}{2} m (1 + (f'(q))^2) \dot{q}^2 $$
**4) Potential Energy:**
$$ U(r) = mg r_y \implies P(q) = mg f(q) $$
**5) Lagrangian:**
$$ \mathcal{L} = K - P = \frac{1}{2} m (1 + (f')^2) \dot{q}^2 - mg f(q) $$
**6) EL Equation (assuming no external applied force, $\tau=0$):**
$$ \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}} \right) - \frac{\partial \mathcal{L}}{\partial q} = 0 $$
First, compute the terms:
$$ \frac{\partial \mathcal{L}}{\partial \dot{q}} = m(1 + (f')^2)\dot{q} $$
$$ \frac{\partial \mathcal{L}}{\partial q} = \frac{1}{2} m \dot{q}^2 (2f'f'') - mgf' = m\dot{q}^2 f'f'' - mgf' $$
Take the time derivative of the first term (requires chain rule for $f'$):
$$ \frac{d}{dt}\left[ m(1 + (f')^2)\dot{q} \right] = m(1 + (f')^2)\ddot{q} + m(2f'f''\dot{q})\dot{q} = m(1 + (f')^2)\ddot{q} + 2m f'f''\dot{q}^2 $$
Substitute back into the EL equation:
$$ \left[ m(1 + (f')^2)\ddot{q} + 2m f'f''\dot{q}^2 \right] - \left[ m\dot{q}^2 f'f'' - mgf' \right] = 0 $$
Combine like terms to get the final equation of motion:
$$ m(1 + (f')^2)\ddot{q} + m f'(q)f''(q)\dot{q}^2 + mgf'(q) = 0 $$

---

## 4. Example 3: RR Robot (2-Link Planar Arm)

**System:** A 2-link robotic arm with link lengths $d_1, d_2$ and tip masses $m_1, m_2$.

**1) Generalized Coordinates:** $n=2$, $q = (q_1, q_2)$. ($q_1$ is the absolute angle of link 1, $q_2$ is the relative angle of link 2).
**2) Parameterization:**
Position of mass 1:
$$ r^1(q) = d_1 \begin{bmatrix} c_{q_1} \\ s_{q_1} \\ 0 \end{bmatrix} $$
Position of mass 2:
$$ r^2(q) = r^1(q) + d_2 \begin{bmatrix} c_{q_1+q_2} \\ s_{q_1+q_2} \\ 0 \end{bmatrix} = \begin{bmatrix} d_1 c_{q_1} + d_2 c_{q_1+q_2} \\ d_1 s_{q_1} + d_2 s_{q_1+q_2} \\ 0 \end{bmatrix} $$

**3) Kinetic Energy ($K$):**
First, find velocities:
$$ \dot{r}^1 = d_1 \dot{q}_1 \begin{bmatrix} -s_{q_1} \\ c_{q_1} \\ 0 \end{bmatrix} $$
$$ \dot{r}^2 = d_1 \dot{q}_1 \begin{bmatrix} -s_{q_1} \\ c_{q_1} \\ 0 \end{bmatrix} + d_2 (\dot{q}_1 + \dot{q}_2) \begin{bmatrix} -s_{q_1+q_2} \\ c_{q_1+q_2} \\ 0 \end{bmatrix} $$
Total Kinetic Energy: $K = \frac{1}{2} m_1 \|\dot{r}^1\|^2 + \frac{1}{2} m_2 \|\dot{r}^2\|^2$

*(Missing Derivation Completion): The notes end with $K = \dots$. Here is the full expansion:*
*   $\|\dot{r}^1\|^2 = d_1^2 \dot{q}_1^2 (s_{q_1}^2 + c_{q_1}^2) = d_1^2 \dot{q}_1^2$
*   $\|\dot{r}^2\|^2 = \left( -d_1 \dot{q}_1 s_{q_1} - d_2(\dot{q}_1+\dot{q}_2)s_{q_1+q_2} \right)^2 + \left( d_1 \dot{q}_1 c_{q_1} + d_2(\dot{q}_1+\dot{q}_2)c_{q_1+q_2} \right)^2$
Expanding and using the trigonometric identity $\cos(\alpha)\cos(\beta) + \sin(\alpha)\sin(\beta) = \cos(\alpha-\beta)$ yields:
*   $\|\dot{r}^2\|^2 = d_1^2 \dot{q}_1^2 + d_2^2(\dot{q}_1+\dot{q}_2)^2 + 2d_1d_2\dot{q}_1(\dot{q}_1+\dot{q}_2)\cos(q_2)$

Therefore, the complete Kinetic Energy is:
$$ K(q, \dot{q}) = \frac{1}{2} m_1 d_1^2 \dot{q}_1^2 + \frac{1}{2} m_2 \left[ d_1^2 \dot{q}_1^2 + d_2^2(\dot{q}_1+\dot{q}_2)^2 + 2d_1d_2\dot{q}_1(\dot{q}_1+\dot{q}_2)\cos(q_2) \right] $$

**4) Potential Energy ($P$):**
$$ U(r) = U_1(r^1) + U_2(r^2) = m_1 g r^1_y + m_2 g r^2_y $$
$$ P(q) = m_1 g d_1 s_{q_1} + m_2 g (d_1 s_{q_1} + d_2 s_{q_1+q_2}) $$

---

## 5. Generalized Forces for Multi-body Systems

For a system with multiple configuration vectors (like $n=2$ in the robot example), the EL equations apply to each generalized coordinate:
$$ \frac{d}{dt} \frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i \quad \text{for } i = 1, \dots, n $$

To compute the generalized torque $\tau$ induced by multiple applied forces $f_a$ acting on multiple points $r$, we stack the vectors:
$$ r = \begin{bmatrix} r^1 \\ \vdots \\ r^N \end{bmatrix}, \quad f_a = \begin{bmatrix} f_a^1 \\ \vdots \\ f_a^N \end{bmatrix} $$

Using block-matrix multiplication, the total generalized force/torque is the linear superposition of the individual projected forces:
$$ \tau = \left[ \frac{\partial r}{\partial q} \right]^T f_a = \left[ \begin{array}{c} \partial r^1/\partial q \\ \hdashline \vdots \\ \hdashline \partial r^N/\partial q \end{array} \right]^T \begin{bmatrix} f_a^1 \\ \vdots \\ f_a^N \end{bmatrix} = \left[ \left(\frac{\partial r^1}{\partial q}\right)^T \dots \left(\frac{\partial r^N}{\partial q}\right)^T \right] \begin{bmatrix} f_a^1 \\ \vdots \\ f_a^N \end{bmatrix} $$
$$ \tau = \sum_{i=1}^N \left( \frac{\partial r^i}{\partial q} \right)^T f_a^i $$

For a 2-body system ($N=2$):
$$ \tau = \tau_1 + \tau_2 $$
where:
*   $\tau_1 = \left( \frac{\partial r^1}{\partial q} \right)^T f_a^1$ is the vector of joint torques induced by a force $f_a^1$ applied at position $r^1$.
*   $\tau_2 = \left( \frac{\partial r^2}{\partial q} \right)^T f_a^2$ is the vector of joint torques induced by a force $f_a^2$ applied at position $r^2$.