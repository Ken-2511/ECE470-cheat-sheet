Here is a well-structured summary of the provided lecture notes on robot dynamics, complete with LaTeX formatting. 

I have carefully reviewed the mathematical derivations. While the overall concepts are correct, there are a few minor algebraic typos and notation inconsistencies in the handwritten text that I have corrected and explained below to ensure the equations are fully self-contained and accurate.

---

# ECE470 Robotics: Dynamics and Equations of Motion

## 1. From Point-Masses to Rigid Bodies
When analyzing a robot, we transition from treating links as point masses to treating them as rigid bodies. 

Consider a collection of $m$ rigid bodies $B_1, \dots, B_m$. Let us define the following parameters for each body $B_i$:
*   $M_i$: Total mass of $B_i$.
*   $r_{ci}^0$: Center of Mass (COM) of $B_i$, expressed in the base frame $O$.
*   $I_i \in \mathbb{R}^{3 \times 3}$: Inertia tensor of $B_i$.
*   $\omega_i^0$: Angular velocity of $B_i$ with respect to frame $O$.

### Total System Energy
*   **Kinetic Energy ($T$):** The sum of translational and rotational kinetic energies for all bodies.
    $$T = \frac{1}{2}\sum_{i=1}^m M_i \| \dot{r}_{ci}^0 \|^2 + \frac{1}{2}\sum_{i=1}^m (\omega_i^0)^T I_i \omega_i^0$$
*   **Potential Energy ($P$ or $U$):** Due to gravity (where $g$ is the gravity vector).
    $$P(q) = -\sum_{i=1}^m M_i g^T r_{ci}^0$$

---

## 2. Equations of Motion via Jacobians
For a robot with $n$ links and $n$ Degrees of Freedom (DOFs), we assume $m=n$. 
Let $q = (q_1, \dots, q_n)$ be the configuration vector of the robot defined by the Denavit-Hartenberg (DH) convention.

To compute kinetic energy, we must express the velocities $\dot{r}_{ci}^0$ and $\omega_i^0$ in terms of the joint positions $q$ and velocities $\dot{q}$. We do this using Jacobians.

### Angular Velocity Jacobian ($J_\omega^i$)
$$\omega_i^0 = J_\omega^i(q) \dot{q}$$
Where $J_\omega^i(q)$ is the angular Jacobian assuming the end-effector is at frame origin $O_i$:
$$J_\omega^i(q) = \left[ \rho_1 z_0^0 \;\; \rho_2 z_1^0 \;\dots\; \rho_i z_{i-1}^0 \;\; 0_{3 \times (n-i)} \right]$$
*Where $\rho_j = 0$ if joint $j$ is Prismatic (P), and $\rho_j = 1$ if joint $j$ is Revolute (R).*

### Linear Velocity Jacobian ($J_v^i$)
Viewing the COM $r_{ci}^0$ as the origin of an end-effector frame, the linear velocity is:
$$\dot{r}_{ci}^0 = J_v^i(q) \dot{q}$$

**Intuition (Assuming all joints are Revolute):**
If all joints are revolute, the $j$-th column of the linear Jacobian relates the rotation around axis $z_{j-1}^0$ to the translation of the COM.
$$J_v^i(q) = \left[ z_0^0 \times r_{ci}^0 \;\;\; z_1^0 \times (r_{ci}^0 - O_1^0) \;\;\dots\;\; z_{i-1}^0 \times (r_{ci}^0 - O_{i-1}^0) \;\;\; 0_{3 \times (n-i)} \right]$$

> ⚠️ **Correction to Notes:** The handwritten notes have a slight visual error in the linear Jacobian formula, writing something resembling $z_{i-1}^0 \times (r_{ci}^0 - (r_{ci}^0 - O_{i-1}^0))$. The correct standard Spong/Craig formulation for the $j$-th column is $z_{j-1}^0 \times (r_{ci}^0 - O_{j-1}^0)$, which is what is written above. 

**Safe Approach (Without Spatial Jacobians):**
If the robot contains prismatic joints, the cross-product intuition falls apart. The safest mathematical approach is to find the position vector $r_{ci}^0(q)$ and compute the velocity symbolically using partial derivatives:
$$r_{ci}^0(q) = O_i^0 + R_i^0 d_i \implies \dot{r}_{ci}^0 = \frac{\partial r_{ci}^0}{\partial q} \dot{q}$$

---

## 3. Kinetic Energy and the Mass Matrix $D(q)$
Assuming we have the Jacobians, we can substitute them into the Kinetic Energy equation. 

> ⚠️ **Correction to Notes:** The first line of the handwritten derivation leaves the second term outside the summation over $i$. The mathematically correct, factored form is shown directly below.

$$K(q, \dot{q}) = \frac{1}{2} \dot{q}^T \underbrace{\left[ \sum_{i=1}^n M_i J_v^i(q)^T J_v^i(q) + J_\omega^i(q)^T I_i J_\omega^i(q) \right]}_{D(q)} \dot{q}$$

Here, **$D(q)$ is the $n \times n$ symmetric mass matrix** (or inertia matrix) of the robot. The Lagrangian is then defined as:
$$\mathcal{L} = K - P = \frac{1}{2}\dot{q}^T D(q) \dot{q} - P(q) = \frac{1}{2} \sum_{i,j}^n d_{ij} \dot{q}_i \dot{q}_j - P(q)$$

---

## 4. Euler-Lagrange (EL) Equations
The equations of motion are derived using the EL equations:
$$\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}_k} \right) - \frac{\partial \mathcal{L}}{\partial q_k} = \tau_k \quad \text{for } k=1, \dots, n$$

**Step 1: Compute the time derivative term**
$$\frac{\partial \mathcal{L}}{\partial \dot{q}_k} = \sum_{j=1}^n d_{kj} \dot{q}_j$$
$$\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}_k} \right) = \sum_{j=1}^n d_{kj} \ddot{q}_j + \sum_{j=1}^n \frac{d}{dt}(d_{kj}) \dot{q}_j$$
Using the chain rule, $\frac{d}{dt}(d_{kj}) = \sum_{i=1}^n \frac{\partial d_{kj}}{\partial q_i} \dot{q}_i$. Substituting this yields:
$$\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}_k} \right) = \sum_{j=1}^n d_{kj} \ddot{q}_j + \sum_{i,j}^n \frac{\partial d_{kj}}{\partial q_i} \dot{q}_i \dot{q}_j$$

**Step 2: Compute the configuration derivative term**
$$\frac{\partial \mathcal{L}}{\partial q_k} = \frac{1}{2} \sum_{i,j}^n \frac{\partial d_{ij}}{\partial q_k} \dot{q}_i \dot{q}_j - \frac{\partial P}{\partial q_k}$$

**Step 3: Combine**
Subtracting Step 2 from Step 1 gives the $k$-th torque:
$$\tau_k = \sum_{j=1}^n d_{kj} \ddot{q}_j + \sum_{i,j}^n \left( \frac{\partial d_{kj}}{\partial q_i} - \frac{1}{2} \frac{\partial d_{ij}}{\partial q_k} \right) \dot{q}_i \dot{q}_j + \frac{\partial P}{\partial q_k}$$

> ⚠️ **Correction to Notes:** In the handwritten notes, the term is written as $\left( \frac{\partial d_{kj}}{\partial q_i} \dot{q}_i \dot{q}_j - \frac{1}{2} \frac{\partial d_{ij}}{\partial q_k} \right) \dot{q}_i \dot{q}_j$. This is an algebraic typo where $\dot{q}_i \dot{q}_j$ was mistakenly left inside the parenthesis while also being factored out. The equation written above is the corrected version. Furthermore, because the sum over $i,j$ is symmetric, we can split $\frac{\partial d_{kj}}{\partial q_i}$ into $\frac{1}{2}(\frac{\partial d_{kj}}{\partial q_i} + \frac{\partial d_{ki}}{\partial q_j})$ to create the standard Christoffel symbols.

### Christoffel Symbols
The coefficients grouping the velocity terms are called the **Christoffel symbols of the first kind**, denoted as $c_{ijk}(q)$. As shown on the blackboard, they are computed through *partial* differentiation of the elements of the mass matrix $D$:
$$c_{ijk}(q) = \frac{1}{2} \left[ \frac{\partial d_{kj}}{\partial q_i} + \frac{\partial d_{ki}}{\partial q_j} - \frac{\partial d_{ij}}{\partial q_k} \right]$$

This allows us to write the scalar equations of motion cleanly as:
$$\sum_{j=1}^n d_{kj} \ddot{q}_j + \sum_{i,j}^n c_{ijk} \dot{q}_i \dot{q}_j + \frac{\partial P}{\partial q_k} = \tau_k \quad \text{for } k=1, \dots, n$$

---

## 5. Matrix Form of the Equations of Motion
The scalar equations can be stacked into a highly compact, generalized matrix-vector format used universally in robotics control:

$$D(q)\ddot{q} + C(q, \dot{q})\dot{q} + \nabla_q P(q) = \tau$$

Where:
*   $D(q)$ is the mass/inertia matrix.
*   $C(q, \dot{q})$ is the Coriolis and centrifugal matrix, whose elements are constructed from the Christoffel symbols:
    $$[C(q, \dot{q})]_{kj} = \sum_{i=1}^n c_{ijk}(q) \dot{q}_i$$
*   $\nabla_q P(q)$ is the gravity vector (often denoted as $G(q)$).
*   $\tau$ is the vector of joint torques/forces.