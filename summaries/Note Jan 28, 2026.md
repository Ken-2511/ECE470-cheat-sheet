Here is a structured summary of the lecture notes, transcribed into Markdown with LaTeX math. 

I have preserved all key definitions, equations, and diagrams' logic. I also added some clarifications, corrected minor notational typos, and **filled in the missing step for finding $\theta_1$** to make the inverse kinematics solution complete.

---

# Inverse Kinematics

**Goal:** Given a desired end-effector pose $H_d = \left[ \begin{array}{c|c} R_d & O_d^0 \\ \hline 0 & 1 \end{array} \right] \in SE(3)$, find the joint variables $q = (q_1 \dots q_n)$ such that the forward kinematics equal the desired pose:
$$H_n^0(q) = H_d$$

## Kinematic Decoupling
Assume a serial robot with $n=6$ joints, where the last three joints form a **spherical wrist** (their axes of rotation intersect at a single point called the wrist center). 

Let $O_c^0$ be the position of the wrist center. We can express the end-effector position $O_6^0$ in terms of the wrist center:
$$O_6^0 = O_c^0 + d_6 z_6^0 = O_c^0 + d_6 R_6^0 \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$

Requiring that $O_6^0 = O_d^0$ (desired position) and $R_6^0 = R_d$ (desired orientation) is equivalent to requiring:
$$O_c^0 = O_d^0 - d_6 R_d \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \quad \text{and} \quad R_6^0 = R_d$$

**Conclusion:** This observation enables the decoupling of the Inverse Kinematics Problem (IKP) into two simpler problems: **Inverse Position** and **Inverse Orientation**.

---

## 1. Inverse Position Kinematics

First, realize that the wrist center position $O_c^0$ only depends on the first three joints: $O_c^0 = O_c^0(q_1, q_2, q_3)$.

**Goal:** Find $(q_1, q_2, q_3)$ such that:
$$O_c^0(q_1, q_2, q_3) = O_d^0 - d_6 R_d \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} = \begin{bmatrix} x_c \\ y_c \\ z_c \end{bmatrix}$$
*Output:* $(q_1, q_2, q_3)$ (Note: For revolute joints, $q_i = \theta_i$).

### Solving for a Specific Anthropomorphic Arm Geometry
Based on the robot diagram provided in the notes (an arm with an offset $d_4$ and links $a_2, d_1$), we can solve for the angles geometrically.

***Missing Step Filled In: Solving for $\theta_1$***
> *The notes jump straight to the 2D planar view, but we first need $\theta_1$. By projecting the wrist center onto the $x_0-y_0$ plane:*
> $$\theta_1 = \text{atan2}(y_c, x_c)$$

#### Solving for $\theta_3$
Project the arm onto a 2D plane (side view). We form a triangle with sides $a_2$, $d_4$, and a hypotenuse. 
Let $s$ and $r$ be the vertical and horizontal distances from the second joint to the wrist center:
*   $s = z_c - d_1$
*   $r = \sqrt{x_c^2 + y_c^2}$

Using the **Law of Cosines** ($c^2 = a^2 + b^2 - 2ab \cos\gamma$):
$$r^2 + s^2 = a_2^2 + d_4^2 - 2a_2 d_4 \cos(\gamma)$$
From the geometry, the interior angle $\gamma$ relates to joint angle $\theta_3$ by $\gamma = \pi - (\theta_3 - \frac{\pi}{2})$. 
Therefore, $\cos(\gamma) = \cos\left(\frac{3\pi}{2} - \theta_3\right) = -\sin(\theta_3)$. Substituting this back:
$$r^2 + s^2 = a_2^2 + d_4^2 + 2a_2 d_4 \sin(\theta_3)$$

Solving for $\sin(\theta_3)$ gives us an intermediate variable $D$:
$$(*) \quad \sin(\theta_3) = \frac{x_c^2 + y_c^2 + (z_c - d_1)^2 - a_2^2 - d_4^2}{2 a_2 d_4} =: D$$

Using the standard two-argument arctangent function `atan2(y, x)` where $y = \sin\theta$ and $x = \cos\theta = \pm\sqrt{1-\sin^2\theta}$:
$$\theta_3 = \text{atan2}\left(D, \pm \sqrt{1 - D^2}\right)$$
*   The $+$ solution corresponds to **Elbow Up**.
*   The $-$ solution corresponds to **Elbow Down**.

#### Solving for $\theta_2$
From the 2D view triangle, $\theta_2$ is the difference between two angles: $\theta_2 = \alpha - \beta$.
*   $\alpha$ is the angle to the wrist center: $\alpha = \text{atan2}(s, r) = \text{atan2}\left(z_c - d_1, \sqrt{x_c^2 + y_c^2}\right)$ *(Note: corrected missing 'c' subscript on y from the handwritten notes).*
*   $\beta$ is the interior angle of the links: $\beta = \text{atan2}(\text{opposite}, \text{adjacent}) = \text{atan2}\left(-d_4 \cos\theta_3, a_2 + d_4 \sin\theta_3\right)$

$$\theta_2 = \text{atan2}\left(z_c - d_1, \sqrt{x_c^2 + y_c^2}\right) - \text{atan2}\left(-d_4 \cos\theta_3, a_2 + d_4 \sin\theta_3\right)$$

---

## 2. Inverse Orientation Problem

We now have $(\theta_1, \theta_2, \theta_3)$. 
**Goal:** Find $(\theta_4, \theta_5, \theta_6)$.

1. Using forward kinematics, compute $H_3^0 = H_1^0(\theta_1) H_2^1(\theta_2) H_3^2(\theta_3)$.
2. Extract the rotation matrix $R_3^0$.
3. Define the desired orientation of the wrist $R_6^3$ as:
   $$R_6^3(\theta_4, \theta_5, \theta_6) = M = (R_3^0)^T R_d$$
   where $M$ is our known input matrix.

### Extracting Angles using Euler Parameterization
The notes assume the spherical wrist generates a rotation matrix identical in structure to the **ZYZ Euler angle parameterization** $(\phi, \theta, \psi) \rightarrow (\theta_4, \theta_5, \theta_6)$. 

By extracting $R_6^3$ from $H_6^3 = H_4^3 H_5^4 H_6^5$, we get a matrix where the relevant terms for inversion are:
$$R_6^3(\theta_4, \theta_5, \theta_6) = \begin{bmatrix} \cdot & \cdot & c_4 s_5 \\ \cdot & \cdot & s_4 s_5 \\ -s_5 c_6 & s_5 s_6 & c_5 \end{bmatrix} = \begin{bmatrix} m_{11} & m_{12} & m_{13} \\ m_{21} & m_{22} & m_{23} \\ m_{31} & m_{32} & m_{33} \end{bmatrix} = M$$
*(where $c_i = \cos\theta_i$ and $s_i = \sin\theta_i$)*

By equating elements of $R_6^3$ to elements of $M$, we get two valid sets of inversion formulas (depending on whether $\sin(\theta_5)$ is positive or negative):

**Solution 1 (Assuming $s_5 > 0$):**
$$\theta_4 = \text{atan2}(m_{23}, m_{13})$$
$$\theta_5 = \text{atan2}(\sqrt{1 - m_{33}^2}, m_{33})$$
$$\theta_6 = \text{atan2}(m_{32}, -m_{31})$$

**Solution 2 (Assuming $s_5 < 0$):**
$$\theta_4 = \text{atan2}(-m_{23}, -m_{13})$$
$$\theta_5 = \text{atan2}(-\sqrt{1 - m_{33}^2}, m_{33})$$
$$\theta_6 = \text{atan2}(-m_{32}, m_{31})$$

**Singular Case:**
> *Note: Case where $m_{13} = m_{23} = 0$ is not included in the formulas above.*
> (If $m_{13}$ and $m_{23}$ are both zero, it implies $\sin(\theta_5) = 0$. This means $\theta_5 = 0$ or $\pi$, which causes axes 4 and 6 to align, resulting in a singularity known as Gimbal Lock. In this state, only the sum or difference of $\theta_4$ and $\theta_6$ can be determined).