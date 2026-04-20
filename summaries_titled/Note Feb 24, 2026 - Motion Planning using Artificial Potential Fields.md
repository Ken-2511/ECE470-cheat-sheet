Here is a structured summary of the lecture notes. 

***

# Robotics: Motion Planning using Artificial Potential Fields

## 1. Review: The Motion Planning Problem
**Goal:** Given a start pose $H_s \in SE(3)$, a final pose $H_f \in SE(3)$, and a time $T > 0$, find a reference signal $q^r(t)$ for $t \in [0, T]$ such that:
1. $H_n^0(q^r(0)) = H_s$
2. $H_n^0(q^r(T)) = H_f$
3. $\forall t \in (0, T)$, if $q = q^r(t)$, the robot does not hit obstacles.

**Idea:** Produce artificial forces at each link of the robot to find waypoints, and then interpolate between them.
First, solve the **Inverse Kinematics Problem (IKP):** Find $q_s, q_f$ such that $H_n^0(q_s) = H_s$ and $H_n^0(q_f) = H_f$.

The overall method is broken into steps:
*   **Step 1:** Design forces $F$ (Attractive and Repulsive).
*   **Step 2:** Use gradient descent to generate waypoints.

---

## 2. Mathematical Preliminaries
Before defining the forces, we review some calculus concepts.

1. **Euclidean Norm:** If $x \in \mathbb{R}^n$, the Euclidean norm of $x$ is:
   $$ \|x\| := \sqrt{x^T x} = \sqrt{x_1^2 + \dots + x_n^2} $$

2. **Jacobian Matrix:** If $f: \mathbb{R}^n \to \mathbb{R}^m$ is differentiable, the Jacobian matrix of $f$ is denoted $\frac{\partial f}{\partial x}(x)$, which is an $m \times n$ matrix.
   $$ \left[ \frac{\partial f}{\partial x} \right]_{ij} = \frac{\partial f_i}{\partial x_j} $$
   *Special case:* For a scalar function $f: \mathbb{R}^n \to \mathbb{R}$, the Jacobian is a $1 \times n$ row vector: $\frac{\partial f}{\partial x} = (\nabla f)^T$.

3. **Chain Rule:** Suppose $f: \mathbb{R}^n \to \mathbb{R}^m$ and $g: \mathbb{R}^p \to \mathbb{R}^n$ are differentiable. Then the composition $f(g(x)) = (f \circ g)(x)$ is also differentiable, and:
   $$ \frac{\partial (f \circ g)}{\partial x} = \frac{\partial f}{\partial x}(g(x)) \frac{\partial g}{\partial x} $$

4. **Jacobian of the Norm:**
   > ⚠️ **CORRECTION REGARDING THE NOTES:** 
   > The handwritten notes contain a blank space and an incorrect partial derivative formulation for the Jacobian of the norm: $\frac{\partial \|x\|}{\partial x_i} = \frac{1}{\sqrt{x_i^2}}$. This is mathematically incorrect. 
   >
   > **Correct Derivation:**
   > Let $f(x) = \|x\| = (x_1^2 + x_2^2 + \dots + x_n^2)^{1/2}$. Using the chain rule for a single component $x_i$:
   > $$ \frac{\partial \|x\|}{\partial x_i} = \frac{1}{2}(x_1^2 + \dots + x_n^2)^{-1/2} \cdot (2x_i) = \frac{x_i}{\|x\|} $$
   > Therefore, the full $1 \times n$ Jacobian row vector is:
   > $$ \frac{\partial \|x\|}{\partial x} = \left[ \frac{x_1}{\|x\|}, \frac{x_2}{\|x\|}, \dots, \frac{x_n}{\|x\|} \right] = \frac{x^T}{\|x\|} $$

---

## 3. Step 1: Force Design (Attractive Potential)
The attractive force acts like a spring pulling the joints toward their goal positions.

First, use forward kinematics to compute the desired origins for each joint $i$ based on the final configuration $q_f$:
$$ \overline{O}_i^0 := O_i^0(q_f) $$

We define the **Attractive Potential Energy** as a quadratic well:
$$ U_{i, att}(O_i^0) = \frac{1}{2} c_i \|O_i^0 - \overline{O}_i^0\|^2 $$

The resulting **Attractive Force** is the negative gradient of the potential:
$$ F_{i, att} = -\nabla U_{i, att} = - \left[ \frac{\partial}{\partial O_i^0} U_{i, att} \right]^T $$

Applying the chain rule (and treating the derivative of the position vector with respect to itself as the identity matrix $I_3$):
$$ F_{i, att} = - \left[ c_i \|O_i^0 - \overline{O}_i^0\| \cdot \frac{(O_i^0 - \overline{O}_i^0)^T}{\|O_i^0 - \overline{O}_i^0\|} \cdot I_3 \right]^T $$
$$ F_{i, att} = -c_i (O_i^0 - \overline{O}_i^0) $$

### Saturated Attractive Force
To prevent the artificial force from becoming dangerously large when the robot is far from the goal, we "saturate" the force. It behaves quadratically (linear force) when near the goal, and linearly (constant force magnitude) when far away.
*(Note: There is a minor typo in the original notes where it says $F_{i, ett}$; it has been corrected to $F_{i, att}$ below).*

$$ F_{i, att}(O_i^0) = \begin{cases} 
-c_i (O_i^0 - \overline{O}_i^0) & \text{if } \|O_i^0 - \overline{O}_i^0\| \le d_i \\ 
-c_i d_i \frac{(O_i^0 - \overline{O}_i^0)}{\|O_i^0 - \overline{O}_i^0\|} & \text{else} 
\end{cases} $$

---

## 4. Step 1: Force Design (Repulsive Potential)
The repulsive potential pushes the robot away from obstacles.

**Definitions and Assumptions:**
*   Denote by $\mathcal{O} \subset \mathbb{R}^3$ the obstacle set.
*   **Convex Set:** A set $S \subset \mathbb{R}^3$ is convex if $\forall p, q \in S$, the line segment joining $p$ and $q$ is strictly contained in $S$.
*   Assume that each connected component of $\mathcal{O}$ is a convex set. 
*   **Idea:** Enclose each connected component within a convex bounding set. Develop a repulsive force $F_{i, rep}$ for each convex connected component of $\mathcal{O}$, then sum them up (superposition). Without loss of generality (WLOG), we focus on $\mathcal{O}$ being a single convex set.

**Theorem (Orthogonal Projection):**
Let $\mathcal{O} \subset \mathbb{R}^3$ be a convex set. Then for each point $p \in \mathbb{R}^3$, there exists a **unique** point $\pi(p) \in \mathcal{O}$ such that the distance $\|p - \pi(p)\|$ is minimal. 
*   $\pi(p)$ is called the orthogonal projection of $p$ onto $\mathcal{O}$.

### Example: Projection onto a Sphere (Ball)
Consider a spherical obstacle: $\mathcal{O} = \{ p \in \mathbb{R}^3 \mid \|p\| \le R \}$.

> ⚠️ **CORRECTION REGARDING THE NOTES:**
> The handwritten notes define the projection as:
> $$ \pi(p) = \begin{cases} R \frac{p}{\|p\|} & \text{if } p \neq 0 \\ p & \text{if } p \in \mathcal{O} \end{cases} $$
> **This logic is flawed.** If a point $p$ is strictly inside the sphere but is not the origin (e.g., $\|p\| = R/2$), the first condition ($p \neq 0$) would push it to the boundary $R \frac{p}{\|p\|}$. However, the theorem states that if a point is already inside the set $\mathcal{O}$, the closest point in $\mathcal{O}$ to $p$ is simply $p$ itself. The conditions provided in the notes overlap and give the wrong result for internal points.
> 
> **Corrected formulation:**
> The projection function should be split by whether the point is *outside* or *inside* the obstacle:
> $$ \pi(p) = \begin{cases} 
p & \text{if } p \in \mathcal{O} \text{ (i.e., } \|p\| \le R) \\ 
R \frac{p}{\|p\|} & \text{if } p \notin \mathcal{O} \text{ (i.e., } \|p\| > R) 
\end{cases} $$