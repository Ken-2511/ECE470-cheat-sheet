Here is a well-structured summary of the lecture notes. 

***

# Robotics: Systems of Point Masses and Holonomic Constraints

## 1. System Definition
Consider a system of $N$ point-masses, indexed as $m_1, \dots, m_N$, operating within an internal Cartesian frame $Oxyz$.

*   Let $r^i = \begin{bmatrix} r_x^i \\ r_y^i \\ r_z^i \end{bmatrix}$ denote the position of mass $m_i$ in the frame.
*   The total state of the system is concatenated into a single vector $r \in \mathbb{R}^{3N}$:
    $$r = (r^1, \dots, r^N) = \begin{bmatrix} r^1 \\ \vdots \\ r^N \end{bmatrix} \in \mathbb{R}^{3N}$$

## 2. Mathematical Foundations
Let $g: \mathbb{R}^n \to \mathbb{R}^m$ be a function. 
*   **Zero Level Set:** Denoted by $g^{-1}(0)$, it is the set of all inputs that map to zero: 
    $$g^{-1}(0) = \{ x \in \mathbb{R}^n \mid g(x) = 0 \}$$
*   **$C^1$ Function:** A function $g$ is $C^1$ if it is continuously differentiable (i.e., it is differentiable and the entries of the Jacobian matrix $\frac{\partial g}{\partial x}$ are continuous).

### Holonomic Constraints
A **holonomic constraint** in a system of $N$ point-masses is a relation of the form $g(r^1, \dots, r^N) = 0$, where $g: \mathbb{R}^{3N} \to \mathbb{R}^\ell$ is a $C^1$ function, and the following property holds:
$$ \forall r \in g^{-1}(0), \quad \text{rank}\left(\frac{\partial g}{\partial r}\right) = \ell \quad \text{(full row rank)} \quad \dots (1) $$

*   **Regular Value:** A function $g \in C^1$ satisfying condition (1) is said to have a regular value at $0 \in \mathbb{R}^\ell$. (This means the full rank property holds for all $r$ giving a value of $g=0$).
*   **Constraint Set ($\mathcal{C}$):** The set where the constraint is satisfied, $g^{-1}(0)$, is denoted as $\mathcal{C}$.

---

### ⚠️ Instructor Correction: The Preimage Theorem and Degrees of Freedom
*The notes state that the definition of regular value/holonomic constraints ties directly to the **Constant Rank Theorem** (or **Preimage Theorem**).*

**Theorem Statement:** Under the stated assumptions, the constraint set $\mathcal{C}$ is an "embedded submanifold" of $\mathbb{R}^{3N}$ of dimension $n := 3N - \ell$. 
*   An *embedded submanifold* is a generalization of the notion of a regular surface in $\mathbb{R}^3$ (a surface without corners that can be smoothly parameterized).
*   **Degrees of Freedom (DOF):** The dimension $n = 3N - \ell$ represents the system's degrees of freedom.

Because $\mathcal{C}$ is a regular surface, we are guaranteed to find parametric functions such that the system can be represented using exactly $n$ variables. 

---

## 3. Representations of the Constraint Set
We can define the constraint set $\mathcal{C}$ in two distinct ways:

1.  **Implicit Representation:**
    $$\mathcal{C} = \{ (r^1, \dots, r^N) \mid g(r^1, \dots, r^N) = 0 \}$$
2.  **Parametric Representation:**
    $$\mathcal{C} = \{ (r^1, \dots, r^N) \mid r^i = \phi^i(q_1, \dots, q_n) \text{ for } (q_1, \dots, q_n) \in \mathbb{R}^n \}$$

Any choice of parameters $(q_1, \dots, q_n)$ representing $\mathcal{C}$ is called a set of **generalized coordinates**.

> **Instructor Clarification on Notation:** In the notes, it is written that we can find functions $\phi^i: \mathbb{R}^n \to \mathbb{R}^{3N}$. To be mathematically precise, the *combined* state function $\phi(q)$ maps to $\mathbb{R}^{3N}$, but an individual particle's mapping $\phi^i(q)$ maps parameters to a 3D coordinate space, so $\phi^i: \mathbb{R}^n \to \mathbb{R}^3$.

---

## 4. Examples

*(Note: The variable $l$ in the notes is ambiguously used for both the "length" of a pendulum and the "number of constraints" $\ell$. To make this summary strictly clear, $L$ will be used for physical length, and $\ell$ will be used for the number of constraints).*

### Example 1: Simple Pendulum
*   **System:** $N=1$ mass on a rigid rod of length $L$, moving in the $xy$-plane.
*   **Constraints ($\ell = 2$):** The length is constant, and there is no movement in the $z$-axis.
    $$g(r^1) = \begin{bmatrix} \|r^1\|^2 - L^2 \\ r_z^1 \end{bmatrix} = 0$$
*   **DOFs:** $n = 3N - \ell = 3(1) - 2 = 1$ DOF.
*   **Generalized Coordinate:** Let $q_1 = \theta$ (angle of the pendulum).
*   **Parametric Form:** $r(q_1) = \begin{bmatrix} L \sin(q_1) \\ -L \cos(q_1) \\ 0 \end{bmatrix}$ *(Assuming angle is measured from the downward vertical axis).*

> 🚨 **Error in the Original Notes:** 
> In image 2, the notes claim that for the pendulum, the Jacobian $\frac{\partial g}{\partial r}$ "has rank 3 on $g^{-1}(0)$". 
> **Correction:** This is mathematically impossible. $g(r)$ is a vector with $2$ equations (length constraint and z-plane constraint). The Jacobian matrix $\frac{\partial g}{\partial r}$ has dimensions $2 \times 3$. The maximum possible rank of a $2 \times 3$ matrix is $2$.
> **Proof:** $\frac{\partial g}{\partial r} = \begin{bmatrix} 2r_x & 2r_y & 2r_z \\ 0 & 0 & 1 \end{bmatrix}$. On the zero level set, $r_z=0$ and $r_x^2 + r_y^2 = L^2$, meaning $r_x$ and $r_y$ cannot be simultaneously zero. Therefore, the two rows are linearly independent, and **the rank is exactly $2$ (full row rank).**

### Example 2: Particle on a Sphere
*   **System:** $N=1$ mass constrained to the surface of a sphere of radius $R$.
*   **Constraints ($\ell = 1$):**
    $$g(r^1) = \|r^1\|^2 - R^2 = 0$$
*   **DOFs:** $n = 3N - \ell = 3(1) - 1 = 2$ DOFs.
*   **Jacobian / Rank:**
    $$\frac{\partial g}{\partial r^1} = 2 \|r^1\| \frac{(r^1)^T}{\|r^1\|} = 2(r^1)^T$$
    *(Note: Using the vector dot product $r^T r = \|r\|^2$, the derivative simplifies directly to $2(r^1)^T$.)*
    Since $r^1 \neq 0$ on the sphere, $\text{rank}\left(\frac{\partial g}{\partial r}\right) = 1$.
*   **Generalized Coordinates:** $q_1, q_2$ (Standard spherical coordinate angles).
*   **Parametric Form:** *(Using $S_q, C_q$ as shorthand for $\sin(q), \cos(q)$)*
    $$r = \phi(q_1, q_2) = R \begin{bmatrix} \sin(q_1)\cos(q_2) \\ \sin(q_1)\sin(q_2) \\ \cos(q_1) \end{bmatrix}$$

### Example 3: Mass on a Roller Coaster
*(Listed as an exam concept in the notes, illustrating a 1D path in 3D space, meaning $N=1, \ell=2 \implies 1 \text{ DOF}$).*

### Example 4: RR (Revolute-Revolute) Planar Manipulator
*   **System:** $N=2$ masses connected by two links of lengths $d_1$ and $d_2$ in a plane.
*   **Constraints ($\ell = 4$):**
    1. Distance from origin to $m_1$ is $d_1$: $\|r^1\|^2 - d_1^2 = 0$
    2. Distance between $m_1$ and $m_2$ is $d_2$: $\|r^2 - r^1\|^2 - d_2^2 = 0$
    3. Mass 1 stays in $xy$-plane: $r_z^1 = 0$
    4. Mass 2 stays in $xy$-plane: $r_z^2 = 0$
    $$g(r^1, r^2) = \begin{bmatrix} \|r^1\|^2 - d_1^2 \\ \|r^2 - r^1\|^2 - d_2^2 \\ r_z^1 \\ r_z^2 \end{bmatrix} = 0$$
*   **DOFs:** $n = 3N - \ell = 3(2) - 4 = 2$ DOFs.
*   **Generalized Coordinates:** $q_1$ (absolute angle of link 1), $q_2$ (relative angle of link 2).
*   **Parametric Form:**
    $$r^1 = \phi^1(q_1, q_2) = \begin{bmatrix} d_1 \cos(q_1) \\ d_1 \sin(q_1) \\ 0 \end{bmatrix}$$
    $$r^2 = \phi^2(q_1, q_2) = r^1 + \begin{bmatrix} d_2 \cos(q_1 + q_2) \\ d_2 \sin(q_1 + q_2) \\ 0 \end{bmatrix}$$