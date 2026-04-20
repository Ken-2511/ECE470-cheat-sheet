Here is a well-structured summary of your ECE470 lecture notes. 

I have carefully reviewed the mathematical derivations. **Please pay special attention to the "Corrections & Clarifications" boxes**, as I have pointed out and corrected a few typographical and mathematical errors present in the original handwritten notes, and filled in a missing logical step in the derivation of the kinetic energy.

---

# Lecture Summary: From Point-Masses to Distributed Mass
**Key Concepts:** Center of Mass, Inertia Tensor, Rigid Body Dynamics

## 1. Notational Transition & Basic Energy Equations
The notes transition from representing a generic mass position $r^i$ to a more specific notation:
*   $r_i^0$: Position of point mass $m_i$ measured in frame 0.

For a system of $N$ discrete point masses, the fundamental energy equations are:
*   **Kinetic Energy ($T$):** $$T = \frac{1}{2} \sum_{i=1}^N m_i \|\dot{r}_i^0\|^2$$
*   **Potential Energy ($U$):** $$U = -\sum_{i=1}^N m_i \vec{g} \cdot r_i^0$$ *(Note: the minus sign assumes the gravity vector $\vec{g}$ points downwards, e.g., $[0,0,-g]^T$, resulting in a positive $+mgz$ potential).*

These are used in the Lagrangian framework:
*   Kinetic energy in joint space: $K(q, \dot{q}) = \left. T \right|_{\dot{r} = \frac{\partial r}{\partial q}\dot{q}}$
*   Potential energy in joint space: $P(q) = U(r(q))$
*   **Lagrangian:** $\mathcal{L} = K - P$

## 2. Center of Mass & Rigid Body Kinematics
Consider one rigid body made up of $N$ point masses.
*   **Center of Mass (COM), $r_c^0$:**
    $$r_c^0 = \frac{\sum_{i=1}^N m_i r_i^0}{\sum_{i=1}^N m_i} = \frac{\sum_{i=1}^N m_i r_i^0}{M}$$
    *(where $M$ is the total mass of the body).*
*   **Relative Position, $d_i^0$:** The position of mass $i$ relative to the COM.
    $$d_i^0 = r_i^0 - r_c^0 \implies r_i^0 = r_c^0 + d_i^0$$
*   **Property of COM:** By definition, the mass-weighted sum of relative distances from the COM is zero:
    $$\sum_{i=1}^N m_i d_i^0 = 0$$

### Defining a Rigid Body Frame
Define a body frame (Frame 1) attached to the COM ($O_1^0 = r_c^0$) with axes $x_1, y_1, z_1$ such that the relative position of any mass $i$ measured in Frame 1 ($d_i^1$) is **constant**. The existence of such a frame defines a rigid body.

The kinematics of mass $i$ can be expressed as:
$$r_i^0 = r_c^0 + d_i^0 = O_1^0 + R_1^0 d_i^1$$
Taking the derivative to find the velocity:
$$\dot{r}_i^0 = \dot{r}_c^0 + \dot{R}_1^0 d_i^1 \quad \text{(because } d_i^1 \text{ is constant)}$$
Using the property $\dot{R}_1^0 = S(\omega_1^0)R_1^0 = \omega_1^0 \times R_1^0$, where $\omega_1^0$ is the angular velocity of Frame 1 w.r.t Frame 0:
$$\dot{r}_i^0 = \dot{r}_c^0 + \omega_1^0 \times (R_1^0 d_i^1) = \dot{r}_c^0 + \omega_1^0 \times d_i^0$$
Using the skew-symmetric matrix property $\omega \times d = -d \times \omega = -S(d)\omega$:
$$\dot{r}_i^0 = \dot{r}_c^0 - S(d_i^0)\omega_1^0$$

## 3. Deriving the Kinetic Energy of a Rigid Body
Substitute the velocity equation into the Kinetic Energy formula:
$$T = \frac{1}{2} \sum_i m_i (\dot{r}_i^0)^T \dot{r}_i^0$$
$$T = \frac{1}{2} \sum_i m_i \left( \dot{r}_c^0 - S(d_i^0)\omega_1^0 \right)^T \left( \dot{r}_c^0 - S(d_i^0)\omega_1^0 \right)$$
Expanding this out yields three terms:
$$T = \underbrace{\frac{1}{2} \sum_i m_i (\dot{r}_c^0)^T \dot{r}_c^0}_{\text{Term 1}} + \underbrace{\frac{1}{2} \sum_i m_i (\omega_1^0)^T S(d_i^0)^T S(d_i^0) \omega_1^0}_{\text{Term 2}} - \underbrace{\sum_i m_i (\omega_1^0)^T S(d_i^0)^T \dot{r}_c^0}_{\text{Term 3}}$$

> **Correction & Missing Logic filled in:**
> The notes omit the explanation of *why* Term 3 disappears. Let's look at Term 3:
> $$-\sum_i m_i (\omega_1^0)^T S(d_i^0)^T \dot{r}_c^0 = -(\omega_1^0)^T \left[ S\left( \sum_i m_i d_i^0 \right)^T \right] \dot{r}_c^0$$
> Because $\sum_i m_i d_i^0 = 0$ (the property of the COM), the entire matrix $S(0) = 0$. Therefore, **Term 3 evaluates exactly to 0.**

Simplifying Term 1 and Term 2:
*   **Term 1:** $\frac{1}{2} (\sum m_i) \|\dot{r}_c^0\|^2 = \frac{1}{2} M \|\dot{r}_c^0\|^2$
*   **Term 2:** Since $S(d_i^0)$ is skew-symmetric, $S(d_i^0)^T = -S(d_i^0)$. Therefore, $S(d_i^0)^T S(d_i^0) = -(S(d_i^0))^2$.

> **Correction on Notation in Notes:** 
> In the notes for Term 2, it is written as $\frac{1}{2} S(\omega_1^0)^T [ \dots ] \omega_1^0$. This is mathematically incorrect because kinetic energy is a scalar, requiring a vector transpose. The $S(\dots)$ should not be there. It should simply be the transpose of the angular velocity vector: **$(\omega_1^0)^T$**. (The notes correctly write it this way on the next page).

**Final Corrected Equation for $T$:**
$$T = \frac{1}{2} M \|\dot{r}_c^0\|^2 + \frac{1}{2} (\omega_1^0)^T \left[ \sum_{i=1}^N -m_i (S(d_i^0))^2 \right] \omega_1^0$$
$$T = \underbrace{\frac{1}{2} M \|\dot{r}_c^0\|^2}_{\text{Translational}} + \underbrace{\frac{1}{2} (\omega_1^0)^T I \omega_1^0}_{\text{Rotational}}$$

## 4. The Inertia Tensor
From the derivation above, the inertia tensor $I$ of the body w.r.t the COM measured in frame 0 is defined as:
$$I = -\sum_{i=1}^N m_i S(d_i^0)^2$$
Expanded into matrix form, this yields the standard symmetric inertia matrix:
$$I = \begin{bmatrix} 
\sum m_i(y_i^2+z_i^2) & -\sum m_i x_i y_i & -\sum m_i x_i z_i \\ 
-\sum m_i x_i y_i & \sum m_i(x_i^2+z_i^2) & -\sum m_i y_i z_i \\ 
-\sum m_i x_i z_i & -\sum m_i y_i z_i & \sum m_i(x_i^2+y_i^2) 
\end{bmatrix}$$

## 5. Multi-Body Systems & Distributed Mass

### Potential Energy of a Rigid Body
$$U(r) = - \sum m_i g^T r_i^0 = - \sum m_i g^T (r_c^0 + d_i^0)$$
Because $\sum m_i d_i^0 = 0$, this simplifies perfectly to:
$$U(r) = - M g^T r_c^0$$

### $m$ Rigid Bodies
For a system with $m$ rigid bodies (where $M_j$ is the total mass and $I_j$ is the inertia tensor of body $j$):

> **Correction on Multi-Body Kinetic Energy formula:**
> The notes contain a factoring error for $T$, placing a $1/2$ both outside the sum *and* in front of the rotational term. The mathematically correct formulation is:
> $$T = \frac{1}{2} \sum_{j=1}^m \left[ M_j \|\dot{r}_{cj}\|^2 + (\omega_j^0)^T I_j \omega_j^0 \right]$$

$$P = - \sum_{j=1}^m M_j \vec{g}^T r_{cj}^0$$

### Continuous Distributed Mass
By taking the limit as $N \to \infty$, point-mass summations become volume integrals over density $\rho(x,y,z)$. The fundamental forms of $T$ and $P$ remain unaffected because they are independent of $N$.
*   $r_c^0 = \frac{1}{M} \int_V \rho(x,y,z) r \, dx dy dz \quad \text{where } r = [x, y, z]^T$
*   $I = -\int_V \rho(x,y,z) S(\Delta r)^2 \, dx dy dz$

---

## 6. Example: RR Robot (2-Link Planar Arm)
Total kinetic energy: $T = T_1 + T_2$
Kinetic energy of link 1: $T_1 = \frac{1}{2} M_1 \|\dot{r}_{c1}^0\|^2 + \frac{1}{2} (\omega_1^0)^T I_1 \omega_1^0$

**Link 1 Kinematics:** (COM is distance $l_{c1}$ from joint 1)
*   $r_{c1}^0 = l_{c1} \begin{bmatrix} c_{q_1} \\ s_{q_1} \\ 0 \end{bmatrix}$
*   $\omega_1^0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \dot{q}_1$

**Link 2 Kinematics:** (Link 1 length is $a_1$, Link 2 COM is $l_{c2}$ from joint 2)
*   $r_{c2}^0 = a_1 \begin{bmatrix} c_{q_1} \\ s_{q_1} \\ 0 \end{bmatrix} + l_{c2} \begin{bmatrix} c_{q_1+q_2} \\ s_{q_1+q_2} \\ 0 \end{bmatrix}$
*   $\omega_2^0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} (\dot{q}_1 + \dot{q}_2)$

**Rotational Kinetic Energy Simplification:**
Because the arm is planar, it only rotates around the z-axis. The matrix multiplication simplifies to selecting the $(3,3)$ element of the inertia tensor (the moment of inertia around the $z_0$ axis):
$$(\omega_1^0)^T I_1 \omega_1^0 = \begin{bmatrix} 0 & 0 & \dot{q}_1 \end{bmatrix} I_1 \begin{bmatrix} 0 \\ 0 \\ \dot{q}_1 \end{bmatrix} = (I_1)_{33} \dot{q}_1^2$$