Here is a well-structured summary of your lecture notes on the Computed Torque Method and the basics of Lyapunov Stability Theory. 

I have transcribed the equations, organized the concepts, and added a few corrections and clarifications where the notes were slightly incomplete or used non-standard notation.

---

# Robotics Lecture Notes Summary: Computed Torque Method & Stability

## 1. The Computed Torque Method (Feedback Linearization)

### Robot Model & Control Specification
The standard dynamics model of a robotic manipulator is given by:
$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + \nabla_q P + K(q)\dot{q} = u \quad (1)$$
*(Note: $\nabla_q P$ represents the gravity vector $g(q)$, and $K(q)\dot{q}$ represents frictional forces).*

*   **Control Specification:** The goal is to make the actual joint positions $q(t)$ track a reference trajectory $q^r(t)$ as $t \to \infty$ for suitable initial conditions.

### Feedback Transformation
To achieve this, we apply a nonlinear feedback transformation (the control input $u$):
$$u = M(q)a + C(q,\dot{q})\dot{q} + \nabla_q P + K(q)\dot{q} \quad (2)$$

Substituting equation (2) into the right-hand side of equation (1):
$$M(q)\ddot{q} = M(q)a \iff \ddot{q} = a \quad (3)$$
**Remark:** The dynamics in equation (3) are now entirely **decoupled** and linear.

### Error Dynamics & PD Control
Define the tracking error as $\tilde{q}_i = q^r_i(t) - q_i$. Taking the second derivative yields:
$$\ddot{\tilde{q}}_i = \ddot{q}^r_i(t) - \ddot{q}_i = \ddot{q}^r_i(t) - a_i$$

To drive the error to zero, we pick $a_i$ using Proportional-Derivative (PD) control with feedforward acceleration:
$$a_i = \ddot{q}^r_i(t) + K_{p,i}\tilde{q}_i + K_{d,i}\dot{\tilde{q}}_i$$

Substituting this into the error equation yields the closed-loop error dynamics:
$$\ddot{\tilde{q}}_i + K_{d,i}\dot{\tilde{q}}_i + K_{p,i}\tilde{q}_i = 0$$

### Choosing the Rate of Convergence
We pick $K_p, K_d > 0$ to ensure the roots of the characteristic polynomial match our desired rate of exponential convergence.
*   **Example:** If we want the error to decay as $\tilde{q}_i(t) \to 0$ as $e^{-ct}$ (for some $c > 0$), the characteristic equation must be $(s+c)^2 = s^2 + 2cs + c^2 = 0$.
*   Therefore, we pick:  **$K_{d,i} = 2c$** and **$K_{p,i} = c^2$**.

### Vectorized Form
We can write $a$ in vector form for all joints simultaneously:
$$a = \ddot{q}^r + K_p\tilde{q} + K_d\dot{\tilde{q}}$$
Where $K_p$ and $K_d$ are diagonal gain matrices:
$$K_p = \begin{bmatrix} k_{p1} & & \\ & \ddots & \\ & & k_{pn} \end{bmatrix}, \quad K_d = \begin{bmatrix} k_{d1} & & \\ & \ddots & \\ & & k_{dn} \end{bmatrix}$$

### System Block Diagram Flow
Based on the sketched diagram, the signal flow of the controller is:
1.  **Reference Input:** The reference trajectory $q^r(t)$ is passed to a summing junction, and its second derivative $\ddot{q}^r$ is generated (via the $s^2$ block).
2.  **Error Calculation:** The actual position $q$ is subtracted from $q^r(t)$ to generate the error $\tilde{q}$.
3.  **PD Controller:** The error passes through the PD block $(K_p + K_d s)$ to create the feedback term.
4.  **Acceleration Command ($a$):** The feedforward term $\ddot{q}^r$ and the PD feedback term are summed to create the auxiliary input $a$.
5.  **Robot Dynamics:** $a$ is passed through the Feedback Transformation (Eq 2) to calculate the actual torques $u$. $u$ is fed to the robot, yielding new states $(q, \dot{q})$, which are extracted and looped back.

---

## 2. Basic Rudiments of Lyapunov Stability Theory

To analyze stability, we first convert the $n$-dimensional 2nd-order robot dynamics into a $2n$-dimensional 1st-order state-space system.

### Defining the State Space
Define the state vector $x \in \mathbb{R}^{2n}$:
$$x = \begin{bmatrix} q \\ \dot{q} \end{bmatrix} = \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$$
*(where $x_1$ and $x_2$ are each vectors of size $n$).*

Taking the derivative gives the nonlinear ODE $\dot{x} = f(x)$:
$$\dot{x} = \begin{bmatrix} \dot{q} \\ \ddot{q} \end{bmatrix} = \begin{bmatrix} x_2 \\ M^{-1}(x_1) \big[ u(x_1,x_2) - C(x_1,x_2) - \nabla_{x_1}P - K(x_1)x_2 \big] \end{bmatrix} \quad (4)$$

> **Instructor / Notation Clarification:** 
> In standard robotics literature, the Coriolis/Centrifugal term is written as a matrix multiplied by velocity: $C(q,\dot{q})\dot{q}$. In your notes, it is written as $C(x_1, x_2)$ inside the bracket. Here, $C(x_1, x_2)$ should be interpreted as the *entire vector term* representing those forces, not just the matrix.

### Equilibria
**Definition:** A vector $\bar{x} \in \mathbb{R}^n$ is an equilibrium point if $f(\bar{x}) = 0$.
**Meaning:** $x(t) = \bar{x}$ is a steady-state solution of equation (4) because $\dot{x} = 0 = f(\bar{x})$.

#### Example: Simple Pendulum
Dynamics: $\ddot{\theta} = -\frac{g}{l}\sin(\theta)$
State vector: $x = \begin{bmatrix} \theta \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$
State derivatives:
$\dot{x}_1 = x_2$
$\dot{x}_2 = -\frac{g}{l}\sin(x_1)$

To find equilibria, set $f(x) = 0$:
$$f(x) = \begin{bmatrix} x_2 \\ -\frac{g}{l}\sin(x_1) \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$
This implies $x_2 = 0$ and $\sin(x_1) = 0 \implies x_1 = k\pi$.
Therefore, the equilibrium points are: $\bar{x} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}, \bar{x} = \begin{bmatrix} \pi \\ 0 \end{bmatrix}$ (and multiples thereof).

---

## 3. Definitions of Stability

Based on the notes and the formal definitions on the blackboard:

*   **Stable:** An equilibrium $\bar{x}$ is stable if for all initial conditions sufficiently close to $\bar{x}$, the solution $x(t)$ stays arbitrarily close to $\bar{x}$.
    *   *(Formal definition from the blackboard):* $\forall \epsilon > 0, \exists \delta > 0$ such that $||x(0) - \bar{x}|| < \delta \implies ||x(t) - \bar{x}|| < \epsilon, \quad \forall t \ge 0$.
*   **Asymptotically Stable (AS):** An equilibrium $\bar{x}$ is asymptotically stable if it is stable AND solutions starting sufficiently close to $\bar{x}$ converge to $\bar{x}$ as $t \to \infty$.
    > **Correction:** Your handwritten note cut off slightly at the end ("solutions suff. close to $\bar{x}$ as $t \to \infty$"). The definition requires that $\lim_{t \to \infty} x(t) = \bar{x}$.

### Definiteness of Functions (Lyapunov Candidates)
A continuous scalar function $V: \mathbb{R}^n \to \mathbb{R}$ is evaluated at an equilibrium point $\bar{x}$:

1.  **Positive Definite (p.d.) at $\bar{x}$:** $V(x) > 0 \quad \forall x \neq \bar{x}$ AND $V(\bar{x}) = 0$.
2.  **Negative Definite (n.d.) at $\bar{x}$:** if $-V$ is positive definite at $\bar{x}$.
3.  **Positive Semidefinite (p.s.d.):** if $V(x) \ge 0$.
4.  **Negative Semidefinite (n.s.d.):** if $V(x) \le 0$.

> **Intuition from the Blackboard:**
> "Idea: generalize the notion of energy using a p.d. function $V$. Then, the derivative of $V$ [$\dot{V}$] signifies dissipation of energy and that 'implies' that the state converges to the minimum energy, the equilibrium $\bar{x}$."