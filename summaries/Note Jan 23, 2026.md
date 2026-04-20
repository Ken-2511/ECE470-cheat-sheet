Here is a structured Markdown summary of the ECE470 (Robotics) lecture notes provided in the images. 

**Note from the Teaching Assistant:** I have carefully reviewed the derivations in these notes. While the first problem is correct, the second problem (cars on a circular track) contains a subtle but critical error regarding coordinate frame handedness (resulting in a reflection matrix instead of a valid rotation matrix) and some notation typos. I have transcribed the notes exactly as written first, and then provided a **"TA Corrections & Additions"** section at the bottom to clarify and fix these issues so your understanding is completely correct.

---

# ECE470 Tutorial: Coordinate Transformation

## Coordinate Transformation of Points
The relationship between a point $P$ expressed in a global frame $O_0$ (as $P^0$) and a local frame $O_1$ (as $P^1$) is given by:

$$P^0 = O_1^0 + P_x^1 x_1^0 + P_y^1 y_1^0 + P_z^1 z_1^0$$
$$P^0 = O_1^0 + R_1^0 P^1$$

This represents **rigid motions** (a generalization of rototranslations).

---

## 2012 Midterm Q1

**Problem Setup:**
Three robots (cars) maintain an equilateral triangle formation with a side length of $1\text{m}$. 
*   The global frame $O_0$ is defined with $x_0$ pointing UP and $y_0$ pointing RIGHT.
*   Given the position of the first robot: $O_1^0 = \begin{bmatrix} t+1 \\ 1 \\ 0 \end{bmatrix}$
*   **Goal:** Find the homogeneous transformation matrices $H_1^0(t)$, $H_2^0(t)$, and $H_3^0(t)$.

**Calculations:**

**1. Frame 1 ($H_1^0$):**
Since the orientation of the robot aligns with the global frame, $R_1^0 = I$.
$$H_1^0 = \left[ \begin{array}{c|c} R_1^0 & O_1^0 \\ \hline 0 & 1 \end{array} \right] = \begin{bmatrix} 1 & 0 & 0 & t+1 \\ 0 & 1 & 0 & 1 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**2. Frame 2 ($H_2^0$):**
Robot 2 is $1\text{m}$ to the right (along the $y_0$ axis) of Robot 1. 
$$O_2^0 = \begin{bmatrix} t+1 \\ 2 \\ 0 \end{bmatrix}$$
$$H_2^0 = \left[ \begin{array}{c|c} R_2^0 & O_2^0 \\ \hline 0 & 1 \end{array} \right] = \begin{bmatrix} 1 & 0 & 0 & t+1 \\ 0 & 1 & 0 & 2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**3. Frame 3 ($H_3^0$):**
Robot 3 is at the top of the equilateral triangle. Using trigonometry (side length 1, angle $\pi/3$):
$$O_3^0 = O_1^0 + \begin{bmatrix} 1 \cdot \sin(\pi/3) \\ 1 \cdot \cos(\pi/3) \\ 0 \end{bmatrix} = \begin{bmatrix} t+1+\frac{\sqrt{3}}{2} \\ 3/2 \\ 0 \end{bmatrix}$$
$$H_3^0 = \left[ \begin{array}{c|c} R_3^0 & O_3^0 \\ \hline 0 & 1 \end{array} \right] = \begin{bmatrix} 1 & 0 & 0 & t+1+\frac{\sqrt{3}}{2} \\ 0 & 1 & 0 & 3/2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

---

## Cars on a Circular Track

**Problem Setup:**
Three robots travel on a circular track of radius $1\text{m}$. They maintain an equilateral triangle formation inscribed within the circle (distance between robots is $\sqrt{3}\text{m}$).
*   Given center of the track: $C^0 = \begin{bmatrix} 2 \\ 2 \\ 0 \end{bmatrix}$
*   Position of Robot 1: $O_1^0(t) = \begin{bmatrix} \cos(t) + 2 \\ \sin(t) + 2 \\ 0 \end{bmatrix}$
*   **Goal:** Find $H_1^0$, $H_2^0$, $H_3^0$.

### Finding $H_1^0$:
We need the rotation matrix $R_1^0 = [x_1^0 \mid y_1^0 \mid z_1^0]$ and translation $O_1^0$.
*   **$x_1^0$ (Forward/Tangent):** The $x_1$ axis is always pointing tangent to the circle, which is the direction of velocity.
    $$\frac{d}{dt}O_1^0 = \begin{bmatrix} -\sin(t) \\ \cos(t) \\ 0 \end{bmatrix}$$
    Letting $S_t = \sin(t)$ and $C_t = \cos(t)$:
    $$\therefore x_1^0 = \frac{\frac{d}{dt}O_1^0}{\left|\frac{d}{dt}O_1^0\right|} = \begin{bmatrix} -S_t \\ C_t \\ 0 \end{bmatrix}$$
*   **$y_1^0$ (Normal):** The notes calculate this pointing radially OUTWARD.
    $$y_1^0 = \frac{O_1^0 - C^0}{|O_1^0 - C^0|} = \begin{bmatrix} C_t \\ S_t \\ 0 \end{bmatrix}$$
*   **Assembling $H_1^0$:**
    $$H_1^0 = \begin{bmatrix} -S_t & C_t & 0 & C_t+2 \\ C_t & S_t & 0 & S_t+2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### Finding $H_2^0$:
To find $H_2^0$, we use the relative transformation: $H_2^0 = H_1^0 H_2^1$.
Because Robot 2 and Robot 1 are relatively stationary, $H_2^1$ is constant.

*   **Rotation $R_2^1$:**
    The notes write the relative rotation as $-\pi/3$ but evaluate the matrix elements for $-2\pi/3$ (see TA note below):
    $$R_2^1 = R_{z, -\pi/3} = \begin{bmatrix} C_\theta & -S_\theta & 0 \\ S_\theta & C_\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} \quad \text{where evaluated } C = -1/2, S = -\sqrt{3}/2$$
*   **Translation $O_2^1$:**
    The chord length is $\sqrt{3}\text{m}$.
    $$O_2^1 = \begin{bmatrix} \sqrt{3}/2 \\ -3/2 \\ 0 \end{bmatrix}$$
*   **Assembling $H_2^1$:**
    $$\therefore H_2^1 = \begin{bmatrix} -1/2 & \sqrt{3}/2 & 0 & \sqrt{3}/2 \\ -\sqrt{3}/2 & -1/2 & 0 & -3/2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

---

# 🛑 TA Corrections & Additions (Crucial for Exam Accuracy)

While transcribing these notes, I noticed two significant mathematical issues in the "Circular Track" problem that will cause points to be deducted on an exam. Here is the breakdown and the correct way to solve it.

### Error 1: Left-Handed Coordinate System in $H_1^0$
In the notes, the student chose $x_1^0$ to be the tangent (velocity) and $y_1^0$ to be the **outward** normal vector. However, they kept $z_1^0 = [0,0,1]^T$. 
If you check the determinant of the student's rotation matrix:
$$\det(R_1^0) = (-S_t)(S_t) - (C_t)(C_t) = -S_t^2 - C_t^2 = -1$$
A valid transformation matrix in $SE(3)$ **must** have $\det(R) = +1$. Because $\det(R) = -1$, the student accidentally created a left-handed "reflection" matrix.

**The Fix:** 
Assuming standard planar robotics convention where $z_1$ remains parallel to the global $z_0$, the $y_1$ axis must point **INWARD** towards the center of the circle to satisfy the right-hand rule ($x \times y = z$).
*   Correct Inward Normal: $y_1^0 = C^0 - O_1^0 = \begin{bmatrix} -C_t \\ -S_t \\ 0 \end{bmatrix}$
*   **Corrected $H_1^0$:**
    $$H_1^0 = \begin{bmatrix} -S_t & -C_t & 0 & C_t+2 \\ C_t & -S_t & 0 & S_t+2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \quad (\text{Now } \det(R_1^0) = 1)$$

### Error 2: Typo in Relative Angle Notation for $R_2^1$
The notes state $R_2^1 = R_{z, -\pi/3}$ (which is $-60^\circ$). This is conceptually wrong: the angle between the *tangents* of two points separated by $120^\circ$ on a circle is $120^\circ$ ($\pm 2\pi/3$), not $60^\circ$. The student confused the angle of the translation chord (which *is* $60^\circ$ from the tangent) with the rotation of the frame itself.

Interestingly, if you look closely at the matrix the student actually calculated for $H_2^1$, they used $\cos(-2\pi/3) = -1/2$ and $\sin(-2\pi/3) = -\sqrt{3}/2$. So, **they wrote the wrong angle in the equation but plugged the correct angle into the final matrix!**

**The Fully Corrected $H_2^1$:**
Using our corrected Frame 1 (where $y_1$ is INWARD), Robot 2 is located $120^\circ$ ahead on the track. 
1.  **Rotation:** The orientation changes by $+120^\circ$ ($2\pi/3$) inward along the curve. 
2.  **Translation:** The chord is $\sqrt{3}\text{m}$ long and sits at a $+60^\circ$ angle relative to $x_1$ (since $y_1$ now points inward toward the chord).
$$H_2^1 = \begin{bmatrix} \cos(120^\circ) & -\sin(120^\circ) & 0 & \sqrt{3}\cos(60^\circ) \\ \sin(120^\circ) & \cos(120^\circ) & 0 & \sqrt{3}\sin(60^\circ) \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} -1/2 & -\sqrt{3}/2 & 0 & \sqrt{3}/2 \\ \sqrt{3}/2 & -1/2 & 0 & 3/2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$