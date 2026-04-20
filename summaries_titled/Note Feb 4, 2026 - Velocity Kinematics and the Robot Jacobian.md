Here is a well-structured Markdown summary of the provided robotics lecture notes. I have used LaTeX for all mathematical expressions and corrected a minor typo found at the end of the notes.

# Robotics Lecture Notes: Kinematics and the Robot Jacobian

## 1. Skew-Symmetric Matrices and Angular Velocity
The mapping from a 3D vector to a skew-symmetric matrix is defined as $S: \mathbb{R}^3 \to so(3)$.

**Definition:** For a physical interpretation of angular velocity, $\omega_1^0 \in \mathbb{R}^3$ is the unique vector such that:
$$S(\omega_1^0) = \dot{R}_1^0 (R_1^0)^T$$
where $S$ is an isomorphism defined as:
$$S\left( \begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix} \right) := \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}$$

### Properties of the function $S$
*   **(i) Linearity:** $S(v_1 + \lambda v_2) = S(v_1) + \lambda S(v_2)$
*   **(ii) Cross Product Equivalence:** $S(v)w = v \times w = -w \times v = -S(w)v$
*   **(iii) Rotation Similarity:** $\forall R \in SO(3), \quad R S(v) R^T = S(Rv)$
    *   *Meaning:* This property implies that $R(v \times w) = (Rv) \times (Rw)$.

---

## 2. Adding Angular Velocity
**Intuition (Superposition):** If we have frames $F_0, F_1, F_2$, the angular velocity of frame 2 relative to frame 0 is the sum of the relative angular velocities, mapped to the same frame:
$$\omega_2^0 = \omega_1^0 + R_1^0 \omega_2^1$$

### Formal Derivation
Given the definition of $S(\omega)$:
1.  $\omega_1^0 \Rightarrow \dot{R}_1^0(R_1^0)^T = S(\omega_1^0) \implies \dot{R}_1^0 = S(\omega_1^0)R_1^0$
2.  $\omega_2^1 \Rightarrow \dot{R}_2^1(R_2^1)^T = S(\omega_2^1) \implies \dot{R}_2^1 = S(\omega_2^1)R_2^1$
3.  $\omega_2^0 \Rightarrow \dot{R}_2^0(R_2^0)^T = S(\omega_2^0)$

Using the rotation composition rule $R_2^0 = R_1^0 R_2^1$, we take the derivative (chain rule):
$$\dot{R}_2^0 = \dot{R}_1^0 R_2^1 + R_1^0 \dot{R}_2^1$$
Substitute the derivatives from steps 1 and 2:
$$\dot{R}_2^0 = S(\omega_1^0)\underbrace{R_1^0 R_2^1}_{R_2^0} + R_1^0 S(\omega_2^1)R_2^1$$

Now, find $S(\omega_2^0)$:
$$S(\omega_2^0) = \dot{R}_2^0 (R_2^0)^T$$
$$S(\omega_2^0) = \left[ S(\omega_1^0)R_2^0 + R_1^0 S(\omega_2^1)R_2^1 \right] (R_2^0)^T$$
$$S(\omega_2^0) = S(\omega_1^0)R_2^0(R_2^0)^T + R_1^0 S(\omega_2^1) \underbrace{R_2^1 (R_2^0)^T}_{(R_1^0)^T}$$
*Note: $R_2^1 (R_2^0)^T = R_2^1 (R_2^1)^T (R_1^0)^T = I (R_1^0)^T = (R_1^0)^T$*

$$S(\omega_2^0) = S(\omega_1^0)I + \underbrace{R_1^0 S(\omega_2^1) (R_1^0)^T}_{\text{Use Property (iii)}}$$
$$S(\omega_2^0) = S(\omega_1^0) + S(R_1^0 \omega_2^1)$$
Using linearity (Property i):
$$S(\omega_2^0) = S(\omega_1^0 + R_1^0 \omega_2^1) \iff \omega_2^0 = \omega_1^0 + R_1^0 \omega_2^1$$

### General Formula for $n$ links
$$\omega_n^0 = \omega_1^0 + R_1^0 \omega_2^1 + R_2^0 \omega_3^2 + \dots + R_{n-1}^0 \omega_n^{n-1}$$
**Notation:** Let $R_0^0 := I_3$. Then we can write this compactly as:
$$\omega_n^0 = \sum_{i=1}^n R_{i-1}^0 \omega_i^{i-1}$$

---

## 3. Robot Jacobian
Forward kinematics maps joint configurations to the end-effector pose in $SE(3)$:
$$F_{kin} : \mathbb{R}^n \to SE(3)$$
$$F_{kin}(q) = H_n^0(q) = \begin{bmatrix} R_n^0(q) & O_n^0(q) \\ 0 & 1 \end{bmatrix}$$

**Position part:** $F_{kin}^{pos}(q) = O_n^0(q)$
To find the linear velocity $\dot{O}_n^0$, we use the chain rule:
$$\dot{O}_n^0 = \frac{\partial O_n^0}{\partial q} \cdot \dot{q}$$
Here, we define the **Linear Velocity Jacobian** $J_v(q)$ (size $3 \times n$):
$$J_v(q) = \frac{\partial O_n^0}{\partial q}$$

For the **rotational part** ($R_n^0$), we want to find the angular velocity $\omega_n^0$.
$$\omega_n^0 = J_\omega(q) \dot{q}$$
Where $J_\omega(q)$ is the **Angular Velocity Jacobian** (size $3 \times n$).

**Twist Vector ($\xi^0$):**
Combining linear and angular velocity yields a $6 \times 1$ twist vector, related to joint velocities by the full $6 \times n$ Jacobian $J(q)$:
$$\xi^0 = \begin{bmatrix} \dot{O}_n^0 \\ \omega_n^0 \end{bmatrix} = \begin{bmatrix} J_v(q) \\ J_\omega(q) \end{bmatrix} \dot{q}$$
$$J(q) = \begin{bmatrix} J_v(q) \\ J_\omega(q) \end{bmatrix}$$

---

## 4. Computation of $J_\omega(q)$
From earlier, $\omega_n^0 = \sum_{i=1}^n R_{i-1}^0 \omega_i^{i-1}$.

The local angular velocity of joint $i$ depends on its type (assuming the joint axis aligns with the local z-axis):
$$\omega_i^{i-1} = \begin{cases} 0_{3 \times 1} & \text{if joint is P (Prismatic)} \\ \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \dot{q}_i & \text{if joint is R (Revolute)} \end{cases}$$

Transforming this to the base frame (frame 0):
$$R_{i-1}^0 \omega_i^{i-1} = \begin{cases} 0 & \text{if P} \\ z_{i-1}^0 \dot{q}_i & \text{if R} \end{cases}$$
*(Because multiplying $R_{i-1}^0$ by the unit z-vector extracts the z-axis of frame $i-1$ expressed in frame 0).*

To write this neatly, let $\rho_i = \begin{cases} 0 & \text{if P} \\ 1 & \text{if R} \end{cases}$.
Then: $R_{i-1}^0 \omega_i^{i-1} = \rho_i z_{i-1}^0 \dot{q}_i$

Expanding the sum for $\omega_n^0$:
$$\omega_n^0 = \rho_1 z_0^0 \dot{q}_1 + \dots + \rho_n z_{n-1}^0 \dot{q}_n = \underbrace{\begin{bmatrix} \rho_1 z_0^0 & \rho_2 z_1^0 & \dots & \rho_n z_{n-1}^0 \end{bmatrix}}_{J_\omega} \dot{q}$$

---

## 5. Computation of $J_v(q)$
The linear velocity is:
$$\dot{O}_n^0 = J_v(q) \dot{q} = \begin{bmatrix} J_v^1 & J_v^2 & \dots & J_v^n \end{bmatrix} \begin{bmatrix} \dot{q}_1 \\ \vdots \\ \dot{q}_n \end{bmatrix} = \sum_{i=1}^n J_v^i(q) \dot{q}_i$$
Where the $i$-th column is $J_v^i(q) = \frac{\partial O_n^0}{\partial q_i}$.

To compute $J_v^i$, assume all joints $q_1, \dots, q_n$ are "frozen" except for $q_i$. Then $\dot{O}_n^0 = J_v^i(q)\dot{q}_i$.

*   **If joint $i$ is P (Prismatic):**
    $q_i$ executes a translation along the axis $z_{i-1}$.
    $$\dot{O}_n^0 = z_{i-1}^0 \dot{q}_i$$
    Comparing this to the equation above $\implies$ **$J_v^i(q) = z_{i-1}^0$**

*   **If joint $i$ is R (Revolute):**
    $q_i$ executes a rotation about the axis $z_{i-1}$ passing through the origin $O_{i-1}$.
    The linear velocity of the end effector (point $O_n^0$) due to this rotation is determined by the cross product $\omega \times r$:
    $$\dot{O}_n^0 = (z_{i-1}^0 \dot{q}_i) \times (O_n^0 - O_{i-1}^0)$$
    Comparing this to the equation above $\implies$ **$J_v^i(q) = z_{i-1}^0 \times (O_n^0 - O_{i-1}^0)$**

> **Correction Note regarding the handwritten notes:**
> In the final image of the notes, the derivation concludes with the line:
> $\dot{O}_n^0 = J_v^i(q) \dot{q}_i = (z_{i-1}^0 \times (O_n^0 - O_{i-1}^0)) \dot{q}_n$
> **This contains a logical typo.** The variable at the very end should be **$\dot{q}_i$**, not $\dot{q}_n$, because we are calculating the velocity contribution specifically generated by the $i$-th joint varying. The corresponding blackboard image in the final photo correctly displays $\dot{q}_i$.
>
> The correct continuous equation is:
> **$\dot{O}_n^0 = J_v^i(q) \dot{q}_i = \left( z_{i-1}^0 \times (O_n^0 - O_{i-1}^0) \right) \dot{q}_i$**