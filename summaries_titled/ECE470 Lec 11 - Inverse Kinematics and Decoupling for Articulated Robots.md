Here is a structured summary of the lecture notes. I have organized the content, transcribed the mathematics into LaTeX, and added a few "TA Notes" to clarify the derivations and ensure the logic is fully self-contained.

# ECE470 Lecture 11: Solution of Inverse Kinematics Problem (IKP)

**Previous Lecture:** Forward Kinematics problem, Introduction to Inverse Kinematics Problem (IKP).
**Today's Topic:** Solution of IKP for the articulated robot.

---

## 1. Kinematic Decoupling for Articulated Robots with Spherical Wrists
For a 6-DOF articulated robot with a spherical wrist, the joint variables are $(q_1, \dots, q_6) = (\theta_1, \dots, \theta_6)$. We use the **kinematic decoupling** approach to separate the problem into two parts: finding the wrist center position, and then finding the wrist orientation.

The goal is to solve for the joints such that the end-effector reaches a desired rotation $R_d$ and desired origin $O_d$:
$$R_6^0 = R_d, \quad O_6^0 = O_d$$

### (a) Inverse Position Kinematics
Find $(\theta_1, \theta_2, \theta_3)$ such that the wrist center $O_c^0$ satisfies:
$$O_c^0(\theta_1, \theta_2, \theta_3) = O_d - d_6 R_d \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
*(Note: The notes also write this equivalently as $O_d - R_d \begin{bmatrix} 0 \\ 0 \\ d_6 \end{bmatrix}$.)*

### (b) Inverse Orientation Kinematics
Find $(\theta_4, \theta_5, \theta_6)$ such that the orientation of the wrist matches the remaining required rotation:
$$R_6^3(\theta_4, \theta_5, \theta_6) = \left[ R_3^0(\theta_1, \theta_2, \theta_3) \right]^T R_d$$

---

## 2. Solving Inverse Position Kinematics

Given the computed wrist center coordinates:
$$\begin{bmatrix} x_c \\ y_c \\ z_c \end{bmatrix} = O_d - d_6 R_d \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
We need to find $\theta_1, \theta_2, \text{ and } \theta_3$.

### Step 1: Finding $\theta_1$
Looking at the projection of the robot on the $x_0-y_0$ plane:
$$\theta_1 = \text{atan2}(y_c, x_c)$$

### Step 2: Defining the Planar Geometry
To find $\theta_2$ and $\theta_3$, we look at the vertical plane containing links 2 and 3. We define two intermediate variables representing the horizontal and vertical distances to the wrist center relative to frame 1:
*   $r = \sqrt{x_c^2 + y_c^2}$
*   $s = z_c - d_1$

### Step 3: Finding $\theta_3$ Using the Law of Cosines
We form a triangle with side lengths $a_2$, $d_4$, and $\sqrt{r^2 + s^2}$. The interior angle opposite to $\sqrt{r^2 + s^2}$ is defined by the geometry as $\gamma = \pi - (\theta_3 - \pi/2)$.

Applying the Law of Cosines ($c^2 = a^2 + b^2 - 2ab \cos\gamma$):
$$(r^2 + s^2) = a_2^2 + d_4^2 - 2a_2 d_4 \cos\left(\pi - \left(\theta_3 - \frac{\pi}{2}\right)\right)$$

> **TA Note on Trigonometry:**
> By trigonometric identities, $\cos(\pi - x) = -\cos(x)$ and $\cos(x - \pi/2) = \sin(x)$. Therefore:
> $\cos\left(\pi - \left(\theta_3 - \frac{\pi}{2}\right)\right) = -\cos\left(\theta_3 - \frac{\pi}{2}\right) = -\sin(\theta_3)$

Substituting this back into the equation:
$$r^2 + s^2 = a_2^2 + d_4^2 + 2a_2 d_4 \sin(\theta_3)$$

Solving for $\sin(\theta_3)$, we define a term $D$:
$$\sin(\theta_3) = \frac{x_c^2 + y_c^2 + (z_c - d_1)^2 - a_2^2 - d_4^2}{2a_2 d_4} =: D$$

Using the `atan2(y, x)` function, which takes the sine component first and the cosine component second, we get two possible solutions (Elbow Up / Elbow Down):
$$\theta_3 = \text{atan2}\left(D, \pm\sqrt{1 - D^2}\right)$$

### Step 4: Finding $\theta_2$
From the geometric diagram, the angle to the vector $(r, s)$ is $\text{atan2}(s, r)$. This angle is the sum of $\theta_2$ and the interior angle of the triangle at the base. 
Using the components of link $d_4$ projected onto the local axes defined by link $a_2$, we find:
$$\theta_2 = \text{atan2}(s, r) - \text{atan2}(-d_4 \cos\theta_3, a_2 + d_4 \sin\theta_3)$$
Where $s = z_c - d_1$ and $r = \sqrt{x_c^2 + y_c^2}$.

### Position Conclusion:
To summarize the inverse position solutions:
1.  $\theta_1 = \text{atan2}(y_c, x_c)$
2.  $\theta_2 = \text{atan2}\left(z_c - d_1, \sqrt{x_c^2 + y_c^2}\right) - \text{atan2}(-d_4 \cos\theta_3, a_2 + d_4 \sin\theta_3)$
3.  $\theta_3 = \text{atan2}\left(D, \pm\sqrt{1 - D^2}\right)$
    *(where $D$ is defined in the box above).*

---

## 3. Solving Inverse Orientation Kinematics

Now we find $(\theta_4, \theta_5, \theta_6)$ such that:
$$R_6^3(\theta_4, \theta_5, \theta_6) = \left[ R_3^0(\theta_1, \theta_2, \theta_3) \right]^T R_d =: M$$
Let $m_{ij}$ be the entries of the known $3 \times 3$ matrix $M$.

Recall the last 3 rows of the DH table corresponding to a spherical wrist:

| Link | $a_i$ | $\alpha_i$ | $d_i$ | $\theta_i$ |
| :---: | :---: | :---: | :---: | :---: |
| 4 | 0 | $-\pi/2$ | $d_4$ | $\theta_4$ |
| 5 | 0 | $\pi/2$ | 0 | $\theta_5$ |
| 6 | 0 | 0 | $d_6$ | $\theta_6$ |

Multiplying the transformation matrices $H_6^3 = H_4^3 H_5^4 H_6^5$, we extract the rotation portion $R_6^3$:
$$R_6^3 = \begin{bmatrix} * & * & c_4 s_5 \\ * & * & s_4 s_5 \\ -s_5 c_6 & s_5 s_6 & c_5 \end{bmatrix}$$
*(where $c_i = \cos\theta_i$ and $s_i = \sin\theta_i$)*

> **TA Note on Euler Angles:** 
> This resultant matrix is precisely the matrix corresponding to standard **ZYZ Euler angles** if we let $\theta_4 = \phi$, $\theta_5 = \theta$, and $\theta_6 = \psi$.

### Finding the Joint Angles
To solve the equation $R_6^3(\theta_4, \theta_5, \theta_6) = M$, we equate the matrix elements. We have two cases depending on whether the wrist is in a singularity.

**Case 1: Standard Configuration ($m_{13}^2 + m_{23}^2 \neq 0$)**
If $m_{13}^2 + m_{23}^2 \neq 0$, this implies $\sin(\theta_5) \neq 0$. We get two distinct solutions (corresponding to the positive and negative roots of $\sin\theta_5$):

**Solution Set A:**
*   $\theta_5 = \text{atan2}\left(\sqrt{1 - m_{33}^2}, m_{33}\right)$
*   $\theta_4 = \text{atan2}\left(m_{23}, m_{13}\right)$
*   $\theta_6 = \text{atan2}\left(m_{32}, -m_{31}\right)$

**Solution Set B:**
*   $\theta_5 = \text{atan2}\left(-\sqrt{1 - m_{33}^2}, m_{33}\right)$
*   $\theta_4 = \text{atan2}\left(-m_{23}, -m_{13}\right)$
*   $\theta_6 = \text{atan2}\left(-m_{32}, m_{31}\right)$

**Case 2: Singular Configuration ($m_{13}^2 + m_{23}^2 = 0$)**
The condition $m_{13}^2 + m_{23}^2 = 0$ corresponds to a physical **kinematic singularity**. In this situation, $\sin(\theta_5) = 0$, meaning joint 5 is at $0$ or $\pi$. The axes of joint 4 and joint 6 align, causing the robot to lose one degree of freedom (it can only achieve a net rotation of $\theta_4 \pm \theta_6$, meaning there are infinitely many solutions for $\theta_4$ and $\theta_6$).