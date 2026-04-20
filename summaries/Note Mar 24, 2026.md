Here is a structured and comprehensive summary of the provided robotics lecture notes.

# Lecture Notes Summary: Robot Modeling and Control

## 1. Modified Robot Dynamics (Adding Motors and Friction)

### Base Robot Model
Previously, the robot dynamics were modeled using the standard Euler-Lagrange equation:
$$D(q)\ddot{q} + C(q, \dot{q})\dot{q} + \nabla_q P = \tau \quad \text{--- (1)}$$
Where:
*   $\tau \in \mathbb{R}^n$: Generalized applied force.
*   $n$: Degrees of Freedom (DoFs).
*   $\tau_i$: Represents joint *torque* for a Revolute (R) joint, and joint *force* for a Prismatic (P) joint.

### Incorporating Actuator Dynamics
Focusing on robots with purely Revolute (R) joints, joint torques are produced by motors. Adding motor dynamics and joint friction modifies the base model. 

The modified robot model is written as:
$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + \nabla_q P + K(q)\dot{q} = u \quad \text{--- (2)}$$

**Key Definitions and Updates:**
*   **Mass Matrix $M(q)$:** The base inertia matrix $D(q)$ is modified to account for the reflected inertia of the motors through the gear train.
    $$M(q) = D(q) + \begin{bmatrix} J_{m_1} r_1^2 & & \\ & \ddots & \\ & & J_{m_n} r_n^2 \end{bmatrix}$$
    Where $J_{m_i}$ is the moment of inertia of the $i$-th motor about its rotating axis, and $r_i$ is the gear reduction ratio of motor $i$.
*   **Control Input $u$:** The applied torque $\tau$ is replaced by the control input $u$. For a DC motor, the input torque at the joint is proportional to the input voltage:
    $$u_i = \frac{k_{m_i}}{R_i} V_i r_i$$
    *(Instructor's note mapping: $k_{m_i}$ is the motor torque constant, $V_i$ is applied voltage, $R_i$ is electrical resistance, and $r_i$ is the gear ratio).*
*   **Friction $K(q)\dot{q}$:** Friction is essentially unknown but is typically modeled as viscous friction (proportional to speed).
    *   $K(q)$ is a matrix where $K = K^T$ and is **positive semi-definite** (i.e., $v^T K v \ge 0$ for all $v$).
    *   *Correction/Clarification:* While the notes write $K(q)$, viscous friction is most commonly modeled as a constant diagonal matrix $K$, independent of configuration $q$.

---

## 2. The Coriolis Matrix Factorization Problem

A critical remark regarding the derivation of the equations of motion:
If we model the robot using the Euler-Lagrange equations directly, we obtain a Coriolis and centrifugal *vector*, often denoted as $c(q, \dot{q})$. We generally do not inherently know the specific factorization matrix $C(q, \dot{q})$ such that $c(q, \dot{q}) = C(q, \dot{q})\dot{q}$. 

*   **Why this matters (Added Context):** There are infinitely many valid $C(q, \dot{q})$ matrices that satisfy this equation. However, for advanced non-linear control (like passivity-based control), we must choose a specific factorization (usually via Christoffel symbols) to ensure that the matrix $(\dot{M}(q) - 2C(q, \dot{q}))$ is **skew-symmetric**.

### Example: 2-DoF RR Robot
The notes show a partial dynamic equation for joint 1 of an RR robot:
$$d_{11}\ddot{q}_1 + d_{12}\ddot{q}_2 - M_2 d_1 l_{c2} \sin(q_2) (2\dot{q}_1\dot{q}_2 + \dot{q}_2^2) + \frac{\partial P}{\partial q_1} = \tau_1$$

*   *Correction on the handwritten notes:* The main handwritten text incorrectly writes the Coriolis term with an extra "2" in front of the mass as `- 2M_2 d_1...`. However, the photo of the blackboard correctly shows it as `- M_2 d_1...`. The standard kinematic derivation confirms the blackboard is correct.
*   Because the term $(2\dot{q}_1\dot{q}_2 + \dot{q}_2^2)$ can be factored into a matrix multiplying $[\dot{q}_1, \dot{q}_2]^T$ in multiple ways, there are "multiple different $C(q, \dot{q})$" matrices for this robot.

---

## 3. Control of Fully Actuated Robots

**Definitions:**
*   **Fully actuated robot:** Every joint has an independent motor ($u \in \mathbb{R}^n$).
*   *Margin Note:* If the control input is modeled as $Bu$ where $B$ is an $n \times k$ matrix and $u$ is $k \times 1$:
    *   If $k < n$, the system is **under-actuated**.
    *   If $k > n$, the system is **over-actuated**.

**The Fundamental Control Problem:**
Given a reference trajectory signal $q^r(t)$ that is twice differentiable, find a feedback controller $u(q, \dot{q}, q^r, \dot{q}^r, \ddot{q}^r)$ that makes the actual trajectory converge to the reference ($q(t) \rightarrow q^r(t)$) under stable conditions.

**Historical Progression of Robot Control Methods:**
1.  **Computed Torque Method** (1972 - 1973) *(Focus of the current lecture)*
2.  **PD Control with Gravity Compensation**
3.  **Passivity-based Controller**
4.  **Adaptive Passivity-based Controller** (Slotine-Li, Ortega-Spong, 1988 - 1989)

*Note: Under-actuated motion planning remains an active area of modern research.*

---

## 4. The Computed Torque Method (Feedback Linearization)

The Computed Torque Method relies on the assumption that we have **perfect knowledge** of the robot's dynamic parameters: the matrix-valued functions $M(q)$, $C(q, \dot{q})$, $P(q)$, and $K(q)$.

Starting with the actual system dynamics:
$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + \nabla_q P(q) + K(q)\dot{q} = u \quad \text{--- (3)}$$

We design a "feedback transformation" control law:
$$u = M(q)a + C(q, \dot{q})\dot{q} + \nabla_q P + K(q)\dot{q} \quad \text{--- (4)}$$
Where $a$ is a new synthetic control input (representing commanded acceleration).

### Mathematical Cancellation
Substitute the control law (4) into the plant dynamics (3):
$$ \underbrace{M(q)\ddot{q} + C(q, \dot{q})\dot{q} + \nabla_q P + K(q)\dot{q}}_{\text{Physics (Plant)}} = \underbrace{M(q)a + C(q, \dot{q})\dot{q} + \nabla_q P + K(q)\dot{q}}_{\text{Engineering (Controller)}}$$

Because we assume perfect parameter knowledge, the nonlinear terms exactly cancel out, leaving:
$$M(q)\ddot{q} = M(q)a$$

Since the mass matrix $M(q)$ is strictly positive definite, it is **always invertible**. Multiplying both sides by $M(q)^{-1}$ yields:
$$\ddot{q} = a$$

### The Resulting System
The highly non-linear, coupled robot dynamics have been perfectly linearized and decoupled into a **Double Integrator Plant**, which is a Linear Time-Invariant (LTI) system. 

### Block Diagram Explanation
The notes illustrate this with a control loop block diagram:
1.  **Outer Loop (Linear Control):** A reference signal $q^r(t)$ is compared to the actual state $q(t)$. A standard linear controller, $C(s)$ (such as a PD controller), processes the error and outputs the commanded acceleration $a$.
2.  **Inner Loop (Feedback Linearization):** The signal $a$, along with state feedback $(q, \dot{q})$, is fed into the non-linear algebraic block defined by Equation (4). This block computes the physical joint torques $u$.
3.  **Robot Dynamics:** The torques $u$ are applied to the physical robot, resulting in motion $(q, \dot{q})$.
4.  **Equivalent LTI Plant:** Because the inner loop exactly cancels the robot's non-linearities, the relationship between the outer-loop command $a$ and the output position $q$ acts exactly like two integrators in series ($\frac{1}{s} \rightarrow \frac{1}{s}$), meaning $\ddot{q} = a$.