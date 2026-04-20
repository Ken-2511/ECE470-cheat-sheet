Here is a comprehensive summary of the lecture notes. I have structured the content, transcribed the equations into LaTeX, and added **TA Notes & Corrections** to fix mathematical errors and fill in missing steps left incomplete in the handwritten notes.

---

# Robotics Midterm Review Notes

## Problem 1: Inverse Kinematics of a PRR Robot (2013 Midterm Q1)

**Given:** A 3-DOF robot with one prismatic joint and two revolute joints (PRR). 
**Goal:** 
(a) Draw the system with Denavit-Hartenberg (DH) frame assignments and build the DH table.
(b) Given the end-effector position $O_3^0 = \begin{bmatrix} a \\ 0 \\ b \end{bmatrix}$, find the joint variables $(q_1, q_2, q_3)$ in terms of $a$ and $b$.

### (a) DH Frame Assignment and Table

Based on the diagram, the $z_0$-axis points upward (direction of prismatic extension $q_1$), and $x_0$ points left. Frame 1 translates up by $q_1$ and rotates such that subsequent revolute joints rotate around $z$-axes pointing out of the page.

| $i$ | $a_i$ | $d_i$ | $\alpha_i$ | $\theta_i$ |
| :---: | :---: | :---: | :---: | :---: |
| **1** | $0.2$ | $q_1$ | $-\frac{\pi}{2}$ | $0$ |
| **2** | $1$ | $0$ | $0$ | $q_2$ |
| **3** | $1$ | $0$ | $0$ | $q_3$ |

### (b) Inverse Kinematics

The notes provide the target position equations extracted from the forward kinematics:
$$ \begin{bmatrix} a \\ 0 \\ b \end{bmatrix} = \begin{bmatrix} 1.2 + \cos(q_2) \\ 0 \\ q_1 - \sin(q_2) \end{bmatrix} $$

Using the geometric approach (drawing a right triangle with angle $q_2$ and hypotenuse $1$), we can isolate $q_2$ and $q_1$:
1. **Solve for $q_2$:**
   $$ \cos(q_2) = a - 1.2 $$
   $$ \sin(q_2) = \pm \sqrt{1 - (a - 1.2)^2} $$
   Using the two-argument arctangent function to find both elbow-up and elbow-down solutions:
   $$ q_2 = \text{atan2}\left( \pm\sqrt{1 - (a - 1.2)^2}, \; a - 1.2 \right) $$

2. **Solve for $q_1$:**
   $$ b = q_1 - \sin(q_2) \implies q_1 = b + \sin(q_2) $$

> **TA Note / Correction:**
> The equations in the notes simplify the robot by essentially ignoring $q_3$ (assuming $q_3 = 0$) and evaluating the position at the second link. If we use the full DH table strictly for $O_3$, the $x$-position would be $0.2 + \cos(q_2) + \cos(q_2 + q_3)$. The notes' equation $x = 1.2 + \cos(q_2)$ assumes that the sum of the horizontal link offsets leading to joint 2 is $1.2$. Regardless, the algebraic isolation of $q_2$ using `atan2` shown in the notes is mathematically sound for the equation provided.

---

## Problem 2: Statics and Jacobian

**Given:** A 2-DOF SCARA-like planar manipulator.
*   Joint 1 is positioned at $\pi/2$ (pointing along global $y_0$).
*   Joint 2 is at an angle of $-\pi/6$. 
*   **Goal:** Find joint torques $\tau_1, \tau_2$ such that the End Effector (EE) applies a specified force $f^0 = \begin{bmatrix} -1 \\ -2 \\ 0 \end{bmatrix}$.

### Jacobian Calculation

The relationship between applied force and joint torque is $\tau = J_v^T f^0$. We must first find the linear velocity Jacobian $J_v$.
$$ J_v = \Big[ z_0^0 \times (O_2^0 - O_0^0) \;\Big|\; z_1^0 \times (O_2^0 - O_1^0) \Big] $$

From the diagrams and standard planar orientation, the axes of rotation are:
$$ z_0^0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}, \quad z_1^0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} $$

Based on the link lengths ($2$m and $3$m) and the given joint states ($q_1 = \pi/2$, $q_2 = -\pi/6$):
$$ O_1^0 = \begin{bmatrix} 0 \\ 2 \\ 1 \end{bmatrix} $$
$$ O_2^0 = O_1^0 + \begin{bmatrix} -3\sin(q_2) \\ 3\cos(q_2) \\ 1 \end{bmatrix} \quad \text{(1 is the relative vertical offset)} $$

> **TA Note / Correction & Completion:**
> The notes cut off mid-calculation and contain a trigonometric mistake. $\cos(-\pi/6) = \sqrt{3}/2$, **not** $\sqrt{2}/2$ as written in the notes. Here is the complete, corrected derivation:
> 
> Evaluate the vector $O_2^0 - O_1^0$ at $q_2 = -30^\circ$:
> $$ O_2^0 - O_1^0 = \begin{bmatrix} -3\sin(-30^\circ) \\ 3\cos(-30^\circ) \\ 1 \end{bmatrix} = \begin{bmatrix} 1.5 \\ 3\sqrt{3}/2 \\ 1 \end{bmatrix} $$
> Which makes $O_2^0 = \begin{bmatrix} 1.5 \\ 2 + 3\sqrt{3}/2 \\ 2 \end{bmatrix}$.
> 
> **Calculate the Jacobian columns:**
> *   Col 1: $z_0^0 \times O_2^0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} 1.5 \\ 2 + 3\sqrt{3}/2 \\ 2 \end{bmatrix} = \begin{bmatrix} -2 - 3\sqrt{3}/2 \\ 1.5 \\ 0 \end{bmatrix}$
> *   Col 2: $z_1^0 \times (O_2^0 - O_1^0) = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} 1.5 \\ 3\sqrt{3}/2 \\ 1 \end{bmatrix} = \begin{bmatrix} -3\sqrt{3}/2 \\ 1.5 \\ 0 \end{bmatrix}$
> 
> **Calculate Torques:**
> $$ \tau = J_v^T f^0 = \begin{bmatrix} -2 - 3\sqrt{3}/2 & 1.5 & 0 \\ -3\sqrt{3}/2 & 1.5 & 0 \end{bmatrix} \begin{bmatrix} -1 \\ -2 \\ 0 \end{bmatrix} $$
> $$ \tau_1 = (-1)\left(-2 - \frac{3\sqrt{3}}{2}\right) + (-2)(1.5) = 2 + \frac{3\sqrt{3}}{2} - 3 = \mathbf{-1 + \frac{3\sqrt{3}}{2}} $$
> $$ \tau_2 = (-1)\left(-\frac{3\sqrt{3}}{2}\right) + (-2)(1.5) = \mathbf{\frac{3\sqrt{3}}{2} - 3} $$

---

## Problem 3: Rigid Body Kinematics (Helicopter/Quadcopter)

**Given:** A moving body (Frame 1) and a spinning propeller (Frame 2). Frame 0 is inertial.
*   Body velocity: $\dot{O}_1^0 = \begin{bmatrix} 1 \\ 1/2 \\ 1/3 \end{bmatrix}$
*   Propeller origin relative to body: $O_2^1 = \begin{bmatrix} -5 \\ 1 \\ 0 \end{bmatrix}$ (Constant)
*   Rotation matrix of body: $R_1^0 = \begin{bmatrix} 0 & -1 & 0 \\ \cos(t) & 0 & -\sin(t) \\ \sin(t) & 0 & \cos(t) \end{bmatrix}$ (where $ct = \cos(t), st = \sin(t)$)
*   Propeller rotation state: $\theta = \pi/2$, $\dot{\theta} = 30$

### 1. Velocity of the Propeller Center ($\dot{O}_2^0$)
The position of the propeller is $O_2^0 = O_1^0 + R_1^0 O_2^1$.
Differentiating with respect to time (noting $\dot{O}_2^1 = 0$):
$$ \dot{O}_2^0 = \dot{O}_1^0 + \dot{R}_1^0 O_2^1 $$
$$ \dot{O}_2^0 = \begin{bmatrix} 1 \\ 1/2 \\ 1/3 \end{bmatrix} + \begin{bmatrix} 0 & 0 & 0 \\ -st & 0 & -ct \\ ct & 0 & -st \end{bmatrix} \begin{bmatrix} -5 \\ 1 \\ 0 \end{bmatrix} = \begin{bmatrix} 1 \\ 1/2 + 5st \\ 1/3 - 5ct \end{bmatrix} $$

> **TA Note / Correction:** In the notes, there is a minor transcription error on the second page where $\dot{O}_2^0$ is written as having $1/3 - 5st$ in the third row. The derivative of $\sin(t)$ is $\cos(t)$, so it should be $1/3 - 5ct$. The student correctly used $-5ct$ in the final part of the problem.

### 2. Velocity of a Point on the Propeller Blade ($\dot{P}^0$)
The position of point P is $P^0 = O_2^0 + R_2^0 P^2$.
Given a point $P^2 = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix}$, we differentiate:
$$ \dot{P}^0 = \dot{O}_2^0 + \dot{R}_2^0 P^2 $$
Where $R_2^0 = R_1^0 R_2^1$ and $R_2^1$ is a standard Z-axis rotation $R_{z,\theta}$.
Expanding the product rule yields the final result shown in the notes:
$$ \dot{P}^0 = \begin{bmatrix} 1 \\ 1/2 + 5st - 30ct \\ 1/3 - 5ct - 30st \end{bmatrix} $$

### 3. Body Angular Velocity ($\omega_1^0$)
The notes pose the question ("WTF $\omega_1^0$") and write the formula $S(\omega_1^0) = \dot{R}_1^0 (R_1^0)^T$, but leave it uncalculated.

> **TA Note / Completion:** Let's perform this calculation to find $\omega_1^0$.
> $$ \dot{R}_1^0 (R_1^0)^T = \begin{bmatrix} 0 & 0 & 0 \\ -\sin(t) & 0 & -\cos(t) \\ \cos(t) & 0 & -\sin(t) \end{bmatrix} \begin{bmatrix} 0 & \cos(t) & \sin(t) \\ -1 & 0 & 0 \\ 0 & -\sin(t) & \cos(t) \end{bmatrix} $$
> Matrix multiplication yields:
> $$ S(\omega_1^0) = \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{bmatrix} $$
> Un-skewing this matrix gives the angular velocity vector:
> $$ \omega_1^0 = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} \text{ rad/s} $$
> *(Intuition check: $R_1^0$ represents a frame swap followed by a constant rotation rate of 1 rad/s around the global X-axis, so this result is exactly what we expect).*