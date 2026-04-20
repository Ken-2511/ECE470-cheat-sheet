Here is a comprehensive summary of the lecture notes on Robotic Modelling. 

***

# Lecture Notes Summary: Robotic Modelling

**Context:** 
*   **Last Time:** Robot Control Part 1 (Decentralized PD control)
*   **Today:** Robotic Modelling
*   **Historical Context:** Newton (1687), Euler (1736), Lagrange (1788)

---

## 1. System Definition: Point-Mass Pendulum
The objective is to model a point-mass pendulum of mass $m$ and length $\ell$.

**Variables and Forces:**
*   Position vector: $r = \begin{bmatrix} r_x \\ r_y \end{bmatrix}$
*   Angle $\theta$: Measured counter-clockwise from the negative y-axis.
*   $f_c$: Constraint force (tension along the pendulum arm).
*   $mg$: Gravitational force (acting downwards).
*   $f_a$: External applied force.

**Newton's Equation of Motion (EoM):**
$$m\ddot{r} = f_c + \begin{bmatrix} 0 \\ -mg \end{bmatrix} + f_a \quad \text{--- (0)}$$

---

## 2. Holonomic Constraints
The mass is constrained to move in a circle of radius $\ell$. This is represented by a **holonomic constraint**:
$$\|r\| = \ell \implies \|r\|^2 = r_x^2 + r_y^2 = \ell^2 \quad \text{--- (1)}$$

Because this must hold for all time $t \in \mathbb{R}$, we can differentiate the constraint with respect to time ($d/dt$):
$$2r_x(t)\dot{r}_x(t) + 2r_y(t)\dot{r}_y(t) = 0 \quad \forall t$$
Dividing by 2 and writing in vector form:
$$\begin{bmatrix} r_x & r_y \end{bmatrix} \begin{bmatrix} \dot{r}_x \\ \dot{r}_y \end{bmatrix} = 0 \quad \forall t \iff r(t) \cdot \dot{r}(t) = 0 \quad \forall t$$
Multiplying by $dt$ yields:
$$r \cdot dr = 0$$

---

## 3. Virtual Displacements & Constraint Forces
Among all possible infinitesimal displacements $dr$ of the mass, only those that satisfy $r \cdot dr = 0$ are compatible with the holonomic constraint.

**Definition:** We denote compatible infinitesimally small displacements as $\delta r$. Therefore, $\delta r$ is a **virtual displacement** such that:
$$r \cdot \delta r = 0$$

**Analyzing the Constraint Force ($f_c$):**
The reaction force $f_c$ is radial (acts along the pendulum arm), meaning it is proportional to the position vector $r$:
$$f_c = \lambda r, \quad \lambda \in \mathbb{R}$$
If we take the dot product of the constraint force with the virtual displacement:
$$f_c \cdot \delta r = (\lambda r) \cdot \delta r = \lambda (r \cdot \delta r) = 0$$
**Observation:** For the pendulum, the reaction (constraint) force is perpendicular to the virtual displacement ($f_c \perp \delta r$).

---

## 4. Generalized Coordinates and Kinematics
It is redundant to represent the position of the mass using two Cartesian numbers ($r_x, r_y$) when it is constrained to a circular path. We only need one angle, $\theta$. 
*   **$\theta$ is called a "generalized coordinate".** By using it, we remove the redundancy in equation (0).

*(Note: The notes use shorthand where $s_\theta = \sin\theta$ and $c_\theta = \cos\theta$.)*

**Kinematics in terms of $\theta$:**
1.  **Position:** $r = \ell \begin{bmatrix} s_\theta \\ -c_\theta \end{bmatrix}$
2.  **Velocity** ($d/dt$): $\dot{r} = \ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \dot{\theta}$
3.  **Acceleration** ($d/dt$): $\ddot{r} = \ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \ddot{\theta} + \ell \begin{bmatrix} -s_\theta \\ c_\theta \end{bmatrix} \dot{\theta}^2 \quad \text{--- (3)}$

### ⚠️ Instructor Correction: Notation of Virtual Displacement
*   **In the notes:** The virtual displacement is written as $\delta r = \ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} d\theta \quad \text{--- (4)}$
*   **Correction:** The notes mix standard differentials ($d\theta$) with virtual displacements ($\delta r$). A virtual displacement $\delta r$ arises from a virtual variation in the generalized coordinate $\delta\theta$. To be mathematically rigorous, equation (4) should be written with $\delta\theta$:
    $$\delta r = \ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \delta\theta$$

---

## 5. Projection of EoM (D'Alembert's Principle)
To eliminate the unknown constraint force $f_c$, we take the dot product of the original Equation of Motion (0) with the virtual displacement $\delta r$. This projects the dynamics onto the direction tangent to the constraint (the circle).

$$m\ddot{r} \cdot \delta r = f_c \cdot \delta r + \begin{bmatrix} 0 \\ -mg \end{bmatrix} \cdot \delta r + f_a \cdot \delta r \quad \text{--- (2)}$$

Since $f_c \cdot \delta r = 0$, the constraint force term disappears. 

### ⚠️ Missing Logical Steps in the Notes: Evaluation of the Dot Product
The notes substitute (3) and (4) into (2) and jump to the final equation. Here is the explicit math to fill in the gaps:

**Substitute kinematics and virtual displacement into the projected EoM:**
$$m \left( \ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \ddot{\theta} + \ell \begin{bmatrix} -s_\theta \\ c_\theta \end{bmatrix} \dot{\theta}^2 \right) \cdot \left( \ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \delta\theta \right) = \begin{bmatrix} 0 \\ -mg \end{bmatrix} \cdot \left( \ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \delta\theta \right) + f_a \cdot \left( \ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \delta\theta \right)$$

**Evaluate the Left Hand Side (LHS):**
*   *Inertial term:* $\left( \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \cdot \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \right) = c_\theta^2 + s_\theta^2 = 1 \implies m\ell^2\ddot{\theta}\delta\theta$
*   *Centrifugal term:* $\left( \begin{bmatrix} -s_\theta \\ c_\theta \end{bmatrix} \cdot \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \right) = -s_\theta c_\theta + c_\theta s_\theta = 0$. *(This term cleanly cancels out).*
*   *LHS Total:* $m\ell^2\ddot{\theta} \delta\theta$

**Evaluate the Right Hand Side (RHS):**
*   *Gravity term:* $\begin{bmatrix} 0 \\ -mg \end{bmatrix} \cdot \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix} \ell \delta\theta = -mg\ell \sin\theta \delta\theta$
*   *Applied force term:* The term $\ell \begin{bmatrix} c_\theta \\ s_\theta \end{bmatrix}$ is the tangent vector. The dot product of the applied force $f_a$ with the tangent vector yields the applied torque $\tau$. Therefore, this term becomes $\ell (f_a)_T \delta\theta$, where $(f_a)_T$ is the tangential component of the applied force.

**Equating LHS and RHS:**
$$m\ell^2\ddot{\theta} \delta\theta = -mg\ell \sin\theta \delta\theta + \ell (f_a)_T \delta\theta$$
Because this must hold true for *any* arbitrary virtual displacement $\delta\theta$, we can factor out and eliminate $\delta\theta$ from both sides.

---

## 6. Final Equation of Motion
By defining the moment of inertia $J = m\ell^2$ and the applied torque $\tau = \ell(f_a)_T$, the resulting equation is:

$$m\ell^2 \ddot{\theta} = -mg\ell \sin\theta + \ell(f_a)_T$$
$$\underbrace{J}_{\text{mom. iner.}} \ddot{\theta} = -mg\ell \sin\theta + \underbrace{\tau}_{\text{applied torque}}$$

**Final Form:**
$$J\ddot{\theta} = -mg\ell\sin\theta + \tau$$

---

## Summary of Core Concepts Covered
1.  **Holonomic Constraint:** A constraint that depends only on coordinates (and possibly time), restricting the system's configuration space (e.g., $\|r\| = \ell$).
2.  **Virtual Displacement:** An infinitesimal displacement ($\delta r$) consistent with the constraints of the system.
3.  **Constraint Forces:** In this example, constraint forces are orthogonal to virtual displacements ($f_c \perp \delta r$).
4.  **Generalized Coordinates:** A minimal set of independent variables (like $\theta$) used to define the configuration of the system, removing redundancies.
5.  **Projection of EoMs:** Projecting Newton's equations along virtual displacements eliminates constraint forces, simplifying the derivation of the equations of motion.