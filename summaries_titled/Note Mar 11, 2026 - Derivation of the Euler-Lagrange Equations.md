Here is a well-structured summary of the handwritten lecture notes. I have organized the derivation into logical sections, formatted all the math using LaTeX, and added clarifying notes where the handwritten derivation skipped steps or contained minor notational errors.

# Derivation of the Euler-Lagrange Equations

These notes cover the derivation of the equations of motion for a robotic system using generalized coordinates and energy methods, ultimately leading to the Euler-Lagrange equations.

## 1. Principle of Virtual Work and Generalized Forces
We assume the derivation starts from D'Alembert's Principle or the Principle of Virtual Work (referred to as equation "0" in the notes). 

Define the **generalized external force** vector $\psi \in \mathbb{R}^n$ as:
$$ \psi = \begin{bmatrix} \psi_1 \\ \vdots \\ \psi_n \end{bmatrix} = \left(\frac{\partial r}{\partial q}\right)^T f_e $$
where:
*   $\frac{\partial r}{\partial q}$ is the system Jacobian (dimension $3N \times n$).
*   $f_e$ is the external force vector in Cartesian space (dimension $3N \times 1$).

The foundational virtual work equation becomes:
$$ (M\ddot{r})^T \frac{\partial r}{\partial q} dq - \psi^T dq = 0 \quad \forall dq \in \mathbb{R}^n $$
Factoring out $dq$:
$$ \left[ (M\ddot{r})^T \frac{\partial r}{\partial q} - \psi^T \right] dq = 0 \quad \forall dq $$
Since this must hold for any arbitrary virtual displacement $dq$, the term inside the brackets must be zero:
$$ (M\ddot{r})^T \frac{\partial r}{\partial q} = \psi^T \quad \text{--- (1)} $$

## 2. Manipulating the Inertial Terms
To connect Equation (1) to kinetic energy, we apply the product rule to the time derivative of a specific quantity. 

*(Note: The notes write $M\ddot{r}\frac{\partial r}{\partial q}$ in the first step below, but dimensionally it must be transposed to a row vector $(M\ddot{r})^T$ to multiply with the Jacobian matrix. This correction has been applied below).*

$$ \frac{d}{dt} \left[ (M\dot{r})^T \frac{\partial r}{\partial q} \right] = (M\ddot{r})^T \frac{\partial r}{\partial q} + (M\dot{r})^T \frac{d}{dt} \left( \frac{\partial r}{\partial q} \right) $$

Using the standard kinematic identity $\frac{d}{dt}\left(\frac{\partial r}{\partial q}\right) = \frac{\partial \dot{r}}{\partial q}$ (noted at the end of the lecture), we can rewrite this as:
$$ \frac{d}{dt} \left[ (M\dot{r})^T \frac{\partial r}{\partial q} \right] = \underbrace{(M\ddot{r})^T \frac{\partial r}{\partial q}}_{\text{Matches LHS of (1)}} + (M\dot{r})^T \frac{\partial \dot{r}}{\partial q} $$

Isolating the first term and substituting it into the Left Hand Side (LHS) of Equation (1) yields:
$$ \underbrace{\frac{d}{dt} \left[ (M\dot{r})^T \frac{\partial r}{\partial q} \right]}_{(a)} - \underbrace{(M\dot{r})^T \frac{\partial \dot{r}}{\partial q}}_{(b)} = \psi^T \quad \text{--- (2)} $$

## 3. Connecting to Kinetic Energy
The total kinetic energy of the system is defined as:
$$ T = \frac{1}{2} \sum_{i=1}^N m_i \|\dot{r}_i\|^2 = \frac{1}{2} \sum_{i=1}^N m_i \dot{r}_i^T \dot{r}_i = \frac{1}{2} \dot{r}^T M \dot{r} $$

Assuming standard kinematics where position depends on generalized coordinates $r = r(q)$, velocity is $\dot{r} = \frac{\partial r}{\partial q}\dot{q}$. We define kinetic energy in terms of generalized coordinates as $K(q, \dot{q})$:
$$ K(q, \dot{q}) := T(\dot{r}) \Big|_{\dot{r} = \frac{\partial r}{\partial q}\dot{q}} $$

Now, let's evaluate partial derivatives of $K$ to match terms $(a)$ and $(b)$ from Equation (2).
*Useful Matrix Calculus Identity:* $\frac{\partial}{\partial x}(x^T P x) = 2(Px)^T$ (for symmetric $P$). Therefore, $\frac{\partial T}{\partial \dot{r}} = \frac{1}{2} \cdot 2 (M\dot{r})^T = (M\dot{r})^T$.

**Evaluating Term (a):**
Using the chain rule:
$$ \frac{\partial K}{\partial \dot{q}} = \frac{\partial T}{\partial \dot{r}} \frac{\partial \dot{r}}{\partial \dot{q}} $$
Since $\dot{r} = \frac{\partial r}{\partial q}\dot{q}$, we know $\frac{\partial \dot{r}}{\partial \dot{q}} = \frac{\partial r}{\partial q}$. Substituting this gives:
$$ \frac{\partial K}{\partial \dot{q}} = (M\dot{r})^T \frac{\partial r}{\partial q} $$
Taking the time derivative perfectly matches term $(a)$:
$$ \frac{d}{dt} \left( \frac{\partial K}{\partial \dot{q}} \right) = \frac{d}{dt} \left[ (M\dot{r})^T \frac{\partial r}{\partial q} \right] $$

**Evaluating Term (b):**
Similarly, taking the partial derivative with respect to $q$:
$$ \frac{\partial K}{\partial q} = \frac{\partial T}{\partial \dot{r}} \frac{\partial \dot{r}}{\partial q} = (M\dot{r})^T \frac{\partial \dot{r}}{\partial q} $$
This perfectly matches term $(b)$.

**Forming the Row-Vector Equation:**
Substituting these back into Equation (2) gives a row-vector equation ($1 \times n$):
$$ \frac{d}{dt} \left( \frac{\partial K}{\partial \dot{q}} \right) - \frac{\partial K}{\partial q} = \psi^T $$

**Converting to Column Vectors (Gradients):**
Taking the transpose of the entire equation converts the partial derivative row vectors into gradient column vectors ($\nabla$):
$$ \frac{d}{dt} \nabla_{\dot{q}} K - \nabla_q K = \psi \quad \text{--- (3)} $$

## 4. Separating Conservative and Applied Forces
The generalized external force $\psi$ can be split into conservative forces (derived from a potential energy $U(r)$) and non-conservative applied forces $f_a$:
$$ f_e = -\nabla_r U(r) + f_a $$
Substituting this into the definition of $\psi$:
$$ \psi = \left(\frac{\partial r}{\partial q}\right)^T f_a - \left(\frac{\partial r}{\partial q}\right)^T \nabla_r U $$

Let $\tau = \left(\frac{\partial r}{\partial q}\right)^T f_a$ be the **generalized applied force** (e.g., joint torques). 

Now, let's simplify the potential energy term. Note that $\nabla_r U = \left( \frac{\partial U}{\partial r} \right)^T$. Using the transpose property $(AB)^T = B^T A^T$ and the chain rule:
$$ \left(\frac{\partial r}{\partial q}\right)^T \nabla_r U = \left(\frac{\partial r}{\partial q}\right)^T \left(\frac{\partial U}{\partial r}\right)^T = \left( \frac{\partial U}{\partial r} \frac{\partial r}{\partial q} \right)^T = \left( \frac{\partial U}{\partial q} \right)^T = \nabla_q P $$
where $P(q) := U(r(q))$ is the potential energy in generalized coordinates.

Thus, the force vector simplifies to:
$$ \psi = \tau - \nabla_q P $$

## 5. The Euler-Lagrange Equations
Substitute the simplified $\psi$ into Equation (3):
$$ \frac{d}{dt} \nabla_{\dot{q}} K - \nabla_q K = \tau - \nabla_q P $$
Rearranging the terms to group the kinetic and potential energies gives:
$$ \frac{d}{dt} \nabla_{\dot{q}} K - \nabla_q K + \nabla_q P = \tau \quad \text{--- (4)} $$

Finally, we define the **Lagrangian** $\mathcal{L}(q, \dot{q})$ as the difference between kinetic and potential energy:
$$ \mathcal{L}(q, \dot{q}) := K(q, \dot{q}) - P(q) $$

Because potential energy $P(q)$ does not depend on velocities $\dot{q}$, we have:
*   $\nabla_{\dot{q}} \mathcal{L} = \nabla_{\dot{q}} K$
*   $\nabla_q \mathcal{L} = \nabla_q K - \nabla_q P$

Substituting these into Equation (4) yields the standard, column-vector form of the **Euler-Lagrange Equations**:
$$ \frac{d}{dt} \nabla_{\dot{q}} \mathcal{L} - \nabla_q \mathcal{L} = \tau $$