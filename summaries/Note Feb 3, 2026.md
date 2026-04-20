Here is a structured and complete Markdown summary of the provided robotics lecture notes. 

***

# Robotics Lecture Notes: Linear & Angular Velocity and Rigid Body Kinematics

## 1. Review of Linear and Angular Velocity
*   $\dot{p}^0(t)$: Linear velocity of point $p(t)$ expressed in frame 0.
*   $\|\dot{p}^0(t)\|$: Speed of the point.
*   **Angular Velocity:** An attribute of a moving frame with respect to (w.r.t.) another frame.

### Definition of Angular Velocity
The angular velocity of frame 1 w.r.t. frame 0, denoted as $\omega_1^0 \in \mathbb{R}^3$, is the unique vector such that:
$$S(\omega_1^0) = \dot{R}_1^0 (R_1^0)^T$$

Where $S(\cdot)$ is the skew-symmetric matrix operator defined as:
$$S\left(\begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}\right) = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}$$

### Velocity of a Point
For a point rotating about a fixed axis with constant origin $o_1^0$ and constant local coordinates $p^1$ (denoted as $p'$ in the notes):
$$\dot{p}^0 = \omega^0 \times p^0 \quad \xrightarrow{\text{generalize}} \quad \dot{p}^0 = \omega_1^0 \times (R_1^0 p^1)$$

**General Case:** If $p$ is a point moving rigidly with frame 1 (i.e., its coordinates in frame 1, $p^1$, are constant), the total velocity in frame 0 is the sum of translational and rotational components:
$$\dot{p}^0 = \dot{o}_1^0(t) + \omega_1^0 \times (R_1^0 p^1)$$
$$\dot{p}^0 = \dot{o}_1^0(t) + S(\omega_1^0) R_1^0 p^1$$

---

## 2. Comprehensive Example: Object in Circular Motion

**Problem Setup:** 
An object (e.g., a drone/plane) travels in a circle of radius $R = 30$ m at a constant height of $100$ m, with a constant speed of $10$ m/s. Frame 1 ($F_1$) is attached to the object, with $x_1$ pointing in the direction of motion. 
*   Given initial condition: At $t = 0$, $x_1 \parallel y_0$.
*   Since the object remains flat, $z_1 = z_0$.

### Step 2.1: Finding the Position and Velocity Vectors
The position of the moving frame's origin $o_1$ w.r.t. frame 0 is:
$$o_1^0(t) = \begin{bmatrix} c_1 + 30\cos(\omega t + \phi) \\ c_2 + 30\sin(\omega t + \phi) \\ 100 \end{bmatrix}$$
*(Note: $c_1, c_2$ define the center of the circle. We differentiate to find velocity, which eliminates them.)*

Taking the time derivative yields the linear velocity:
$$\dot{o}_1^0(t) = \begin{bmatrix} -30\omega\sin(\omega t + \phi) \\ 30\omega\cos(\omega t + \phi) \\ 0 \end{bmatrix}$$

Using the given speed ($10$ m/s) to solve for the angular scalar $\omega$:
$$\text{Speed} = \|\dot{o}_1^0(t)\| = \sqrt{(-30\omega\sin(\dots))^2 + (30\omega\cos(\dots))^2} = \sqrt{900\omega^2(1)} = 30\omega$$
$$30\omega = 10 \implies \omega = \frac{1}{3} \text{ rad/sec}$$

Substituting $\omega = 1/3$ back into the velocity equation:
$$\dot{o}_1^0(t) = \begin{bmatrix} -10\sin\left(\frac{t}{3} + \phi\right) \\ 10\cos\left(\frac{t}{3} + \phi\right) \\ 0 \end{bmatrix}$$

### Step 2.2: Using Initial Conditions to Find $\phi$
We are given that at $t=0$, the $x_1$ axis is parallel to $y_0$. Because the object moves forward along its $x_1$ axis, its velocity vector $\dot{o}_1^0$ must be proportional to $x_1$. Therefore, at $t=0$, $\dot{o}_1^0$ must be proportional to the $y_0$ unit vector:
$$\dot{o}_1^0(0) \propto \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}$$

Evaluating our velocity equation at $t=0$:
$$\dot{o}_1^0(0) = \begin{bmatrix} -10\sin(\phi) \\ 10\cos(\phi) \\ 0 \end{bmatrix}$$

For this vector to be purely in the positive $y$ direction, the $x$-component must be 0 and the $y$-component must be positive:
$$-10\sin(\phi) = 0 \implies \sin(\phi) = 0$$
$$10\cos(\phi) > 0 \implies \cos(\phi) = 1$$
This dictates that $\phi = 0$.

The final velocity equation is:
$$\dot{o}_1^0(t) = \begin{bmatrix} -10\sin(t/3) \\ 10\cos(t/3) \\ 0 \end{bmatrix}$$

### Step 2.3: Finding the Rotation Matrix $R_1^0$
To find the angular velocity vector $\omega_1^0$, we first need the rotation matrix $R_1^0 = \begin{bmatrix} x_1^0 & y_1^0 & z_1^0 \end{bmatrix}$.

1.  **Find $x_1^0$**: It aligns with the direction of velocity.
    $$x_1^0 = \frac{\dot{o}_1^0}{\|\dot{o}_1^0\|} = \begin{bmatrix} -\sin(t/3) \\ \cos(t/3) \\ 0 \end{bmatrix}$$
2.  **Find $z_1^0$**: The object is flat, so $z_1$ aligns with $z_0$.
    $$z_1^0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
3.  **Find $y_1^0$**: Use the right-hand rule ($y_1^0 = z_1^0 \times x_1^0 = -x_1^0 \times z_1^0$). 
    *Note: The notes use matrix multiplication $S(z_1^0)x_1^0$ to compute the cross product.*
    $$y_1^0 = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix} \begin{bmatrix} -\sin(t/3) \\ \cos(t/3) \\ 0 \end{bmatrix} = \begin{bmatrix} -\cos(t/3) \\ -\sin(t/3) \\ 0 \end{bmatrix}$$

Constructing the rotation matrix (using $S_{t/3} = \sin(t/3)$ and $C_{t/3} = \cos(t/3)$ for brevity):
$$R_1^0(t) = \begin{bmatrix} -S_{t/3} & -C_{t/3} & 0 \\ C_{t/3} & -S_{t/3} & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
*(Sanity check: $\det(R_1^0) = (-S)(-S) - (C)(-C) = \sin^2 + \cos^2 = 1$. This is a valid rotation matrix).*

### Step 2.4: Computing Angular Velocity $\omega_1^0$
Using the definition $S(\omega_1^0) = \dot{R}_1^0 (R_1^0)^T$:

$$ \dot{R}_1^0(t) = \begin{bmatrix} -\frac{1}{3}C_{t/3} & \frac{1}{3}S_{t/3} & 0 \\ -\frac{1}{3}S_{t/3} & -\frac{1}{3}C_{t/3} & 0 \\ 0 & 0 & 0 \end{bmatrix} $$

$$ S(\omega_1^0) = \begin{bmatrix} -\frac{1}{3}C_{t/3} & \frac{1}{3}S_{t/3} & 0 \\ -\frac{1}{3}S_{t/3} & -\frac{1}{3}C_{t/3} & 0 \\ 0 & 0 & 0 \end{bmatrix} \begin{bmatrix} -S_{t/3} & C_{t/3} & 0 \\ -C_{t/3} & -S_{t/3} & 0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & -1/3 & 0 \\ 1/3 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix} $$

By comparing this result to the definition of a skew-symmetric matrix, we extract the angular velocity vector:
$$\omega_1^0 = \begin{bmatrix} 0 \\ 0 \\ 1/3 \end{bmatrix}$$

---

## 3. Physical Interpretation of Angular Velocity

Consider a fixed frame $F_0$ and a rigid body $B$ that **rotates without translating**. 
*   "Without translating" means there exists a point $o_1$ in $B$ that doesn't move: $\dot{o}_1^0(t) = 0$ (constant).
*   "Rigid body" means there exists a frame $F_1$ attached to the body such that for all points $p \in B$, the local coordinates $p^1$ are constant.

Let $\omega_1^0$ be the angular velocity of $F_1$ w.r.t. $F_0$.
Let $\ell$ be the **axis of rotation**: the line containing the vector $\omega_1^0$ and passing through the origin $o_1$.

### Kinematic Properties:
For any point $p \in B$ where $p^1$ is constant and $o_1^0$ is constant, the velocity is purely rotational:
$$\dot{p}^0 = \omega_1^0 \times (R_1^0 p^1)$$

Because of the cross product, the velocity vector is always perpendicular to the angular velocity vector:
$$\dot{p}^0 \perp \omega_1^0$$
**Observation 1 (O1):** $\dot{p}^0$ lies in the plane orthogonal to the axis $\ell$ and passing through $p$. In notation: $\forall p \in B, \dot{p}^0 \perp \ell$.

### Points lying on the Axis of Rotation ($\ell$):
Let $p \in B \cap \ell$ (a point on the body that also lies on the axis of rotation). 
The geometric definition of the line $\ell$ is:
$$\ell = \{ p^0 \in \mathbb{R}^3 \mid p^0 = o_1^0 + \lambda \omega_1^0, \lambda \in \mathbb{R} \}$$
So for a point on this line, $p^0(t) = o_1^0 + \lambda(t) \omega_1^0(t)$.

We also know from standard kinematics that $p^0 = o_1^0 + R_1^0 p^1$. Setting these equal gives:
$$R_1^0 p^1 = \lambda \omega_1^0$$

Substitute this back into the rotational velocity equation:
$$\dot{p}^0 = \omega_1^0 \times (R_1^0 p^1) = \omega_1^0 \times (\lambda \omega_1^0) = \lambda(\omega_1^0 \times \omega_1^0) = 0$$

**Observation 2 (O2):** Any point of $B$ lying on $\ell$ at time $t$ has zero velocity at time $t$. In notation: $\forall p \in B \cap \ell, \dot{p}^0 = 0$.

### Conclusion
Observations O1 and O2 define the physical reality that the line $\ell(t)$ is the **instantaneous axis of rotation** of body $B$. Points on this axis do not move, and all other points rotate in planes orthogonal to it.