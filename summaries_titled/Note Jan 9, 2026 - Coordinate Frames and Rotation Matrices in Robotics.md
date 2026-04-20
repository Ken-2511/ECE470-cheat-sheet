Here is a structured summary of the lecture notes, transcribed into Markdown with LaTeX for mathematical formatting. 

*Note: I have added a few "TA Notes" to clarify non-standard axis assignments used in the examples, ensuring the derivations make logical sense.*

---

# Tutorial: Brief Review of Robotics Fundamentals

## 1. Geometric Entities
*   **Points:** Geometric entities used to indicate a **position** in space.
    *   *Example:* For a multi-link robot, the Centers of Mass (CoM) of each link are represented as points.
*   **Vectors:** Geometric entities used to indicate a **direction with a magnitude**.
    *   *Example:* A vector $v$ can indicate linear velocity. Its length (magnitude) is called the *speed*.

## 2. Coordinate Frames and Representations
*   Frames are labeled with indices $0, 1, \dots$
*   **Base Frame (Frame 0):** This is the only stationary frame and it defines the coordinate system of the task space.
*   **Coordinate Representation:**
    Fix a frame $i$ with origin $O_i$ and axes $X_i, Y_i, Z_i$. 
    
    *   **For a vector $v$:** 
        Express $v$ as a linear combination of the frame's axes: $v = v_x X_i + v_y Y_i + v_z Z_i$
        The coordinate representation of $v$ in frame $i$ is:
        $$v^i := \begin{bmatrix} v_x \\ v_y \\ v_z \end{bmatrix}$$
    
    *   **For a point $P$:**
        Express $P$ relative to the origin: $P = O_i + p_x X_i + p_y Y_i + p_z Z_i$
        The coordinate representation of $P$ in frame $i$ is:
        $$P^i := \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix} \in \mathbb{R}^3$$

---

## 3. Example 1: Planar PR (Prismatic-Revolute) Robot

**Problem Setup:**
We have a robot with a prismatic base extending a distance of $1$, and a revolute arm of length $\frac{3}{2}$ positioned at an angle of $\frac{\pi}{4}$. A vector $v$ pointing horizontally from the end-effector has a magnitude of $|v| = 0.5$.
*Goal: Find $P^0$ and $v^0$.*

*(Convention used: $S_\theta = \sin\theta$, $C_\theta = \cos\theta$)*

> 📝 **TA Note on Axis Assignment:** Look closely at the axis triad in the notes' diagram. The axes are defined unconventionally:
> *   $z_0$ points to the **right** (horizontal).
> *   $x_0$ points **up** (vertical).
> *   $y_0$ points **out of the page** (to satisfy the right-hand rule $x \times y = z$).
> The math below is correct based *only* on this specific axis orientation.

**Finding $P^0$:**
*   Distance along $x_0$ (vertical): The arm extends upward by $\frac{3}{2} \sin(\frac{\pi}{4})$.
*   Distance along $y_0$ (out of page): Planar motion, so $0$.
*   Distance along $z_0$ (horizontal): Base extension ($1$) + horizontal arm projection ($\frac{3}{2} \cos(\frac{\pi}{4})$).

$$P^0 = O_0 + \left(\frac{3}{2}S_{\frac{\pi}{4}}\right) X_0 + (0) Y_0 + \left(1 + \frac{3}{2}C_{\frac{\pi}{4}}\right) Z_0$$
$$P^0 = \begin{bmatrix} \frac{3}{2}S_{\frac{\pi}{4}} \\ 0 \\ 1 + \frac{3}{2}C_{\frac{\pi}{4}} \end{bmatrix} = \begin{bmatrix} \frac{3}{4}\sqrt{2} \\ 0 \\ 1 + \frac{3}{4}\sqrt{2} \end{bmatrix}$$

**Finding $v^0$:**
The vector $v$ points to the right (purely along the $z_0$ axis) with a magnitude of 0.5.
$$v = 0 X_0 + 0 Y_0 + 0.5 Z_0 \implies v^0 = \begin{bmatrix} 0 \\ 0 \\ 0.5 \end{bmatrix}$$

---

## 4. Rotations and Rotation Matrices

A rotation matrix $R_i^0$ represents the rotation of frame $i$ with respect to frame $0$. It is constructed by projecting the axes of frame $i$ onto frame $0$.
$$R_i^0 = \begin{bmatrix} X_i^0 & \vdots & Y_i^0 \end{bmatrix} \quad \text{(can also be found using dot products)}$$

**Uses for rotation matrices:**
1.  Change the coordinate representation of vectors from one frame to another.
2.  Use them as "rotators" (devices to actively rotate vectors within a given frame).

### Example 2: 2D Rotation Matrix Derivation
> 📝 **TA Note on Frame Definition:** The derivation in the notes defines a very specific, non-standard relationship between frame 0 ($X_0, Y_0$) and frame 1 ($X_1, Y_1$). Based on the equations, the angle $\theta$ represents the angle from the $X_0$ axis to the $Y_1$ axis. The math below correctly derives the matrix for this exact visual definition.

**Given definitions:**
$$X_1 = S_\theta X_0 - C_\theta Y_0$$
$$Y_1 = C_\theta X_0 + S_\theta Y_0$$

**Finding $R_1^0$ by definition:**
$$R_1^0 = \begin{bmatrix} X_1^0 & \vdots & Y_1^0 \end{bmatrix}$$
Using the coefficients from the definitions above:
$$R_1^0 = \begin{bmatrix} S_\theta & C_\theta \\ -C_\theta & S_\theta \end{bmatrix}$$

---

### Example 3: 3D Rotation Matrices for a Planar Arm
Consider a standard 2D planar arm with joints $\theta_1$ and $\theta_2$. Even though it moves in 2D, we can represent it using 3D frames.
*Goal: Find $R_1^0, R_2^1,$ and $R_2^0$.*

**Finding $R_1^0$:**
Because it is a standard planar robot, the $Z$ axes all point out of the page:
$$Z_0 = Z_1 = Z_2 \implies Z_1^0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$

Deriving $X_1$ and $Y_1$ relative to Frame 0 (standard standard counter-clockwise rotation):
$$X_1 = C_{\theta_1} X_0 + S_{\theta_1} Y_0 \implies X_1^0 = \begin{bmatrix} C_{\theta_1} \\ S_{\theta_1} \\ 0 \end{bmatrix}$$
$$Y_1 = -S_{\theta_1} X_0 + C_{\theta_1} Y_0 \implies Y_1^0 = \begin{bmatrix} -S_{\theta_1} \\ C_{\theta_1} \\ 0 \end{bmatrix}$$

Putting it together:
$$R_1^0 = \begin{bmatrix} C_{\theta_1} & -S_{\theta_1} & 0 \\ S_{\theta_1} & C_{\theta_1} & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**Finding $R_2^1$ and $R_2^0$:**
By the same logic, the rotation of frame 2 relative to frame 1 is:
$$R_2^1 = \begin{bmatrix} C_{\theta_2} & -S_{\theta_2} & 0 \\ S_{\theta_2} & C_{\theta_2} & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

The total rotation from the base to frame 2 is simply the sum of the angles $\theta_1 + \theta_2$:
$$R_2^0 = \begin{bmatrix} C_{\theta_1+\theta_2} & -S_{\theta_1+\theta_2} & 0 \\ S_{\theta_1+\theta_2} & C_{\theta_1+\theta_2} & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**Checking the Math:**
You can verify that composition of rotations holds true: $R_1^0 \cdot R_2^1 = R_2^0$. 
This multiplication relies on the following standard trigonometric identities:
*   $\sin(\theta_1 + \theta_2) = S_{\theta_1}C_{\theta_2} + C_{\theta_1}S_{\theta_2}$
*   $\cos(\theta_1 + \theta_2) = C_{\theta_1}C_{\theta_2} - S_{\theta_1}S_{\theta_2}$