Here is a well-structured Markdown summary of your robotics lecture notes. 

As requested, I have carefully reviewed the math and logic. The notes contain several incomplete vectors, notation inconsistencies, and **major algebraic errors in the final potential energy calculation**. I have transcribed the notes as written, but I have added a **"TA Corrections & Clarifications"** section at the bottom to fix these mistakes so your study material is accurate and self-contained.

---

# ECE470 Robotics: Equations of Motion & Modeling Strategies

## 1. Equations of Motion (EoMs)
The general form of the equations of motion for a robot is:
$$D(q)\ddot{q} + C(q,\dot{q})\dot{q} + \nabla_q P = \tau$$

Where the elements of the Coriolis/Centrifugal matrix $C$ are given by:
$$[C]_{kj} = \sum_{i=1}^n c_{ijk} \dot{q}_i$$
*(Note: Calculating this requires computing $n^3$ terms).*

## 2. Modeling Strategies

### Step 1: Find Kinetic Energy $K(q, \dot{q})$
The goal is to find kinetic energy in the form: $K(q,\dot{q}) = \frac{1}{2}\dot{q}^T D(q) \dot{q}$. There are two main methods to find the inertia matrix $D(q)$:

**Method 1a: Kinematic Derivation**
1. Start from the standard rigid body kinetic energy equations.
2. Express the linear velocity $\dot{r}_{c_i}^\circ$ and angular velocity $\omega_i^\circ$ in terms of $q$ and $\dot{q}$.
    *   For $\omega_i^\circ$: You can use the Jacobian identity $\omega_i^\circ = J_\omega^i \dot{q}$, or in simple cases, directly derive the expression.
    *   For $\dot{r}_{c_i}^\circ$: Find the position vector $r_{c_i}^\circ(q)$, then take the time derivative $\frac{d}{dt}$ using the chain rule.
3. Extract the elements of $D(q)$ from the kinetic energy expression:
    $$ \dot{q}_i : \begin{cases} \text{terms } \frac{1}{2}\dot{q}_i^2 \rightarrow d_{ii} \\ \text{terms } \frac{1}{2}\dot{q}_i\dot{q}_j \rightarrow \frac{1}{2}d_{ij} \end{cases} $$

**Method 1b: Direct Jacobian Method**
Directly write the inertia matrix using the linear and angular Jacobians:
$$D(q) = \sum_i m_i J_v^i(q)^T J_v^i(q) + J_\omega^i(q)^T I_i J_\omega^i(q)$$

### Step 2: Find Potential Energy $P(q)$
The gravitational potential energy is found using the dot product of the gravity vector $\vec{g}^\circ$ and the center of mass positions $r_{c_i}^\circ(q)$:
$$P = -\sum_{i=1}^n m_i \vec{g}^{\circ T} r_{c_i}^\circ(q)$$

### Step 3: Formulate the EoMs
You can formulate the final equations of motion using one of two methods:

**Method 3a: Direct Matrix Formula**
Use the standard formula $D(q)\ddot{q} + C(q,\dot{q})\dot{q} + \nabla_q P = \tau$.
1. Using the $D$ matrix, compute the $n^3$ Christoffel symbols of the first kind:
   $$c_{ijk}(q) = \frac{1}{2} \left[ \frac{\partial d_{kj}}{\partial q_i} + \frac{\partial d_{ki}}{\partial q_j} - \frac{\partial d_{ij}}{\partial q_k} \right]$$
2. Populate the matrix $C$ using $[C]_{kj} = \sum_{i=1}^n c_{ijk}(q) \dot{q}_i$
3. Compute the gravity vector $\nabla_q P$

**Method 3b: Euler-Lagrange Equations**
Form the Lagrangian $\mathcal{L} = K - P$ and apply the Euler-Lagrange equations directly:
$$\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}_k} \right) - \frac{\partial \mathcal{L}}{\partial q_k} = \tau_k \quad \text{for } k=1 \dots n$$

---

## Example: Cart Pendulum with Spring Rod
*(Note: Refer to the TA corrections below regarding definitions and typos in the handwritten notes).*

**System Parameters:**
*   $k$: spring constant
*   $l$: rest displacement (length) of spring
*   $M_1$: mass rod 1
*   $M_2$: mass rod 2
*   $I_1$: inertia tensor rod 1
*   $I_2$: inertia tensor rod 2

### Using Method 1a (Kinematics to KE)
**Position Vectors ($r_{c_i}^\circ$):**
$$r_{c_1}^\circ = \begin{bmatrix} 0 \\ 0 \\ q_1 \end{bmatrix}, \quad r_{c_2}^\circ = \begin{bmatrix} a_1 + l_c s_{q_2} \\ q_1 + d_1 + l_c c_{q_2} \end{bmatrix}, \quad r_{c_3}^\circ = \begin{bmatrix} a_1 + (l_c+q_3) s_{q_2} \\ 0 \\ q_1 + d_1 + (l_c+q_3) s_{q_2} \end{bmatrix}$$

**Angular Velocity Vectors ($\omega_i^\circ$):**
$$\omega_1^\circ = 0_{3 \times 1}, \quad \omega_2^\circ = \omega_3^\circ = \begin{bmatrix} 0 \\ \dot{q}_2 \\ 0 \end{bmatrix}$$

**Kinetic Energy ($K$):**
Taking the time derivatives of positions and calculating $K = \sum \frac{1}{2}mv^2 + \frac{1}{2}\omega^T I \omega$:
$$
\begin{aligned}
K = \frac{1}{2} M_1 \dot{q}_1^2 &+ \frac{1}{2} M_2 \left[ (l_c c_{q_2} \dot{q}_2)^2 + (\dot{q}_1 - l_c s_{q_2} \dot{q}_2)^2 \right] \\
&+ \frac{1}{2} M_3 \left[ (\dot{q}_3 s_{q_2} + (l_c+q_3) c_{q_2} \dot{q}_2)^2 + (\dot{q}_1 + \dot{q}_3 c_{q_2} - (l_c+q_3) s_{q_2} \dot{q}_2)^2 \right] \\
&+ \frac{1}{2} \dot{q}_2^2 \left( (I_1)_{22} + (I_2)_{22} \right)
\end{aligned}
$$

**Potential Energy ($P$):**
Given the upward $x$-axis, gravity is defined as $\vec{g}^\circ = \begin{bmatrix} -g & 0 & 0 \end{bmatrix}^T$.
$$P = -M_2 \vec{g}^{\circ T} r_{c_2}^\circ - M_3 \vec{g}^{\circ T} r_{c_3}^\circ + \frac{1}{2} k (q_3 - l)^2$$
*(As written in notes - see TA corrections for the correct final algebraic form):*
$$= -M_2 g \left( a_1 + l_c s_{q_2} + a_1 + (l_c+q_3) s_{q_2} \right) + \frac{1}{2} k (q_3 \cdot l)^2$$

---

## 🚨 TA Corrections & Clarifications 🚨

While tracing through the example in the notes, several critical errors and sloppy notations were found. Please use the corrected versions below for your studies:

**1. Ambiguous Mass & Inertia Definitions:**
The notes define $M_1$ as "mass rod 1", but the math clearly treats $M_1$ as the **cart** moving along $q_1$. Furthermore, the KE equation suddenly introduces $M_3$. 
*   **Correction:** $M_1$ is the mass of the cart. $M_2$ is the mass of the pendulum rod. $M_3$ is the point mass attached to the end of the spring.

**2. Missing Coordinates in Position Vectors:**
The cart moves along the $z$-axis ($q_1$), and gravity acts along the $x$-axis. This requires 3D vectors. The notes mistakenly write $r_{c_2}^\circ$ as a 2x1 vector.
*   **Correction:** $r_{c_2}^\circ$ must include a zero for the $y$-axis to be mathematically valid:
    $$r_{c_2}^\circ = \begin{bmatrix} a_1 + l_c \sin(q_2) \\ 0 \\ q_1 + d_1 + l_c \cos(q_2) \end{bmatrix}$$

**3. Geometric Typo in $r_{c_3}^\circ$:**
In the notes, the top row and bottom row of $r_{c_3}^\circ$ both use $\sin(q_2)$ (written as $s_{q_2}$). This is geometrically impossible for resolving a vector. The kinetic energy derivation implies the correct variable was $\cos(q_2)$.
*   **Correction:** The $z$-component of $r_{c_3}^\circ$ should use cosine:
    $$r_{c_3}^\circ = \begin{bmatrix} a_1 + (l_c+q_3) \sin(q_2) \\ 0 \\ q_1 + d_1 + (l_c+q_3) \cos(q_2) \end{bmatrix}$$

**4. MAJOR ERROR: Final Potential Energy Calculation:**
The final line of the potential energy derivation in the notes contains severe algebraic mistakes: it factors $M_2$ and $M_3$ together as just $M_2$, has an incorrect overall negative sign, and mangles the spring potential term at the end into $(q_3 \cdot l)^2$. 

*   **Derivation of the Correction:**
    Since $\vec{g}^\circ = [-g, 0, 0]^T$, the dot product $-\vec{g}^{\circ T} r_{c_i}^\circ$ simplifies to $+g \cdot (x\text{-component})$. Therefore, the gravitational potential is purely positive.
*   **Corrected Potential Energy Equation:**
    $$P = + M_2 g \left( a_1 + l_c \sin(q_2) \right) + M_3 g \left( a_1 + (l_c+q_3) \sin(q_2) \right) + \frac{1}{2}k(q_3 - l)^2$$