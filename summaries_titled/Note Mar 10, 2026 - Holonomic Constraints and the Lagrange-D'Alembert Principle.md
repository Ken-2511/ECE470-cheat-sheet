Here is a structured summary of the lecture notes. 

***

# Robotics: Holonomic Constraints and the Lagrange-D'Alembert Principle

## 1. System of $N$ Masses and Holonomic Constraints
Consider a system of $N$ point-masses where $r^i \in \mathbb{R}^3$ represents the position of mass $m_i$. The state of the entire system can be stacked into a single vector:
$$r = \begin{bmatrix} r^1 \\ \vdots \\ r^N \end{bmatrix} \in \mathbb{R}^{3N}$$

**Holonomic Constraints** are relations of the form:
$$g(r) = g(r^1, \dots, r^N) = 0$$
where $g: \mathbb{R}^{3N} \to \mathbb{R}^\ell$ is continuously differentiable ($C^1$) and has full row rank: $\text{rank } \frac{\partial g}{\partial r} = \ell \quad \forall r \in g^{-1}(0)$.

*   **Constraint Set:** The set of all valid configurations is defined as $\mathcal{C} = g^{-1}(0)$.
*   **Degrees of Freedom (DOFs):** The number of degrees of freedom is $n = 3N - \ell$.

### The Constant Rank Theorem
By the Constant Rank Theorem, the set $\mathcal{C}$ is an "embedded submanifold" of $\mathbb{R}^{3N}$ of dimension $n$. Therefore, we can parameterize $\mathcal{C}$ (or patches of $\mathcal{C}$) using $n$ variables. 

We can represent $\mathcal{C}$ in two equivalent ways:
1.  **Implicit Representation:** $\mathcal{C} = \{r \in \mathbb{R}^{3N} \mid g(r) = 0\}$
2.  **Parametric Representation:** $\mathcal{C} = \{r \in \mathbb{R}^{3N} \mid r = \phi(q), \; q \in \mathbb{R}^n\}$

Any set of variables $q = \begin{bmatrix} q_1, \dots, q_n \end{bmatrix}^T$ used to parameterize $\mathcal{C}$ is called a set of **generalized coordinates** for the mechanical system.

---

## 2. Tangent Vectors to $\mathcal{C}$
The **tangent space** to $\mathcal{C}$ at a point $r \in \mathcal{C}$ is denoted as $T_r\mathcal{C}$. It is the set of all vectors tangent to the surface $\mathcal{C}$ at point $r$. 

We can characterize a tangent vector $v \in T_r\mathcal{C}$ in two equivalent ways:

### (A) Using the Implicit Representation
$$T_r\mathcal{C} = \left\{ v \in \mathbb{R}^{3N} \;\middle|\; \frac{\partial g}{\partial r}(r) \cdot v = 0 \right\} = \text{Ker}\left(\frac{\partial g}{\partial r}\right)$$
The tangent vectors exist in the null space of the Jacobian of the constraints.

> **Example:** A particle on a sphere surface defined by $g(r) = r^T r - R^2 = 0$.
> *   $\frac{\partial g}{\partial r} = 2r^T$
> *   The gradient is $\nabla g = \left[\frac{\partial g}{\partial r}\right]^T = 2r$.
> *   A tangent vector $v$ is orthogonal to the gradient ($\nabla g \perp v$), so $2r^T v = 0 \iff r \perp v$.

### (B) Using the Parametric Representation
$$T_r\mathcal{C} = \left\{ v \in \mathbb{R}^{3N} \;\middle|\; v = \frac{\partial \phi}{\partial q} \cdot w, \quad w \in \mathbb{R}^n \right\} = \text{Im}\left(\frac{\partial \phi}{\partial q}\right)$$
*(Note: The handwritten notes write $w \in \mathbb{R}^N$ here, but it should logically be $\mathbb{R}^n$ where $n$ is the number of DOFs, since the Jacobian $\frac{\partial \phi}{\partial q}$ has dimensions $3N \times n$.)*

> **Example:** A 3D sphere parameterized by angles $q_1, q_2$:
> $$ \mathcal{C} = \left\{ r \in \mathbb{R}^3 \mid r = R \begin{bmatrix} \sin(q_1)\cos(q_2) \\ \sin(q_1)\sin(q_2) \\ \cos(q_1) \end{bmatrix} = \phi(q_1, q_2) \right\} $$
> A tangent vector $v \in T_r\mathcal{C}$ is formed by a linear combination of the column vectors of the Jacobian:
> $$ v = \frac{\partial \phi}{\partial q} w = \begin{bmatrix} \frac{\partial \phi}{\partial q_1} & \frac{\partial \phi}{\partial q_2} \end{bmatrix} \begin{bmatrix} w_1 \\ w_2 \end{bmatrix} = \sum_{i=1}^2 w_i \frac{\partial \phi}{\partial q_i} $$

---

## 3. Virtual Displacements
**Definition:** A **virtual displacement** $\delta r$ at point $r$ is simply a tangent vector $\delta r \in T_r\mathcal{C}$.

Using our two characterizations, we can define $\delta r$ as:
*   **[A] Implicit:** $\frac{\partial g}{\partial r} \cdot \delta r = 0_{\ell \times 1}$
    *   *(Dimensions: $\ell \times 3N$ multiplied by $3N \times 1$ yields an $\ell \times 1$ zero vector).*
*   **[B] Parametric:** $\delta r = \frac{\partial \phi}{\partial q} \cdot \delta q$ for some virtual coordinate displacement $\delta q \in \mathbb{R}^n$.
    *   *(Dimensions: $3N \times n$ multiplied by $n \times 1$).*

> **Example (2D Pendulum):**
> *   **Implicit:** $g(r) = r^T r - \ell^2$. Virtual displacement requires $2r^T \delta r = 0 \implies r \cdot \delta r = 0$.
> *   **Parametric:** $\mathcal{C} = \left\{ r = \phi(q) = \ell \begin{bmatrix} \sin q \\ -\cos q \end{bmatrix} \right\}$. Virtual displacement is $\delta r = \frac{\partial \phi}{\partial q} \delta q = \ell \begin{bmatrix} \cos q \\ \sin q \end{bmatrix} \delta q$.

*(Note: While for a pendulum $\delta r \perp r$ holds true, the notes specifically warn that **it is not true in general that $\delta r \perp r$** for all systems. The universally correct characterizations are always A or B).*

---

## 4. Mechanical System Dynamics & Lagrange-D'Alembert Principle

Returning to Newton's equations for the individual masses, the equation of motion for mass $m_i$ is:
$$m_i \ddot{r}^i = f_c^i + f_e^i$$
Where $f_c^i$ is the constraint force and $f_e^i$ is the external force.

Stacking these equations for all $N$ masses into block matrices and vectors:
$$ r = \begin{bmatrix} r^1 \\ \vdots \\ r^N \end{bmatrix}, \quad f_c = \begin{bmatrix} f_c^1 \\ \vdots \\ f_c^N \end{bmatrix} \in \mathbb{R}^{3N}, \quad f_e = \begin{bmatrix} f_e^1 \\ \vdots \\ f_e^N \end{bmatrix} \in \mathbb{R}^{3N} $$
$$ M = \begin{bmatrix} m_1 I_3 & & \\ & m_2 I_3 & \\ & & \ddots \end{bmatrix} \in \mathbb{R}^{3N \times 3N} $$
*(Note: The notes write $M = \mathbb{R}^{3N \times 3N}$, but it should be $M \in \mathbb{R}^{3N \times 3N}$).*

Newton's equation for the full system becomes:
$$ M\ddot{r} = f_c + f_e $$
$$ M\ddot{r} - f_c - f_e = 0 $$

If we take the dot product of this system with any virtual displacement $\delta r \in T_r\mathcal{C}$:
$$ (M\ddot{r} - f_c - f_e)^T \delta r = 0 $$
$$ (M\ddot{r})^T \delta r - f_c^T \delta r - f_e^T \delta r = 0 \quad \forall \delta r \in T_r\mathcal{C} $$
*(Correction: The notes write $M\ddot{r}^T \delta r$. Due to matrix dimension rules, the transpose of the stacked column vector $M\ddot{r}$ must be written as $(M\ddot{r})^T$ or $\ddot{r}^T M$. Since $M$ is symmetric, $\ddot{r}^T M \delta r$ is the technically correct matrix multiplication form).*

### The Lagrange-D'Alembert Principle
Based on experimental evidence, we make a fundamental assumption about constraint forces (e.g., tension in a pendulum rod, normal forces on a surface):
> **Assumption:** The constraint forces $f_c$ do no virtual work. 
> $$ f_c^T \delta r = 0 \quad \forall \delta r \in T_r\mathcal{C} $$

This assumption is called the **Lagrange-D'Alembert Principle**. Because constraint forces act normal to the constraint surface, and virtual displacements exist on the tangent surface, their dot product is zero. 

**Multi-body Example ($N=2$):**
If a system has two particles ($N=2$), the total virtual displacement vector is stacked:
$$ \delta r = \begin{bmatrix} \delta r^1 \\ \delta r^2 \end{bmatrix} \in \mathbb{R}^6 $$
The implicit constraint condition $\frac{\partial g}{\partial r} \delta r = 0$ still universally applies to the stacked system to ensure the displacement does not violate the holonomic constraints.