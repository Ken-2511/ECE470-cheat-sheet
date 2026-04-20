Here is a comprehensive, well-structured Markdown summary of the lecture notes provided. I have formalized the mathematics using LaTeX, double-checked the derivations, and added brief clarifying notes where the mathematical logic was incomplete (specifically in the singularity case of the inverse Euler angle problem).

---

# ECE470 Robotics: Lecture Notes Summary

## 1. Composition of Rotational Transformations: ZYZ Euler Angles

The notes introduce the composition of rotational transformations, focusing on **ZYZ Euler Angles**, which is the most important case for this course. 

A set of three angles $(\phi, \theta, \psi)$ can be used to represent any rotation matrix $R \in SO(3)$. In robot kinematics, the ZYZ Euler angles perfectly coincide with the joint angles of a **spherical wrist**.

### Forward Kinematics for ZYZ Euler Angles
To achieve the final orientation Frame 1 ($F_1$) from the base Frame 0 ($F_0$):
1. Rotate $F_0$ by angle $\phi$ around $z_0 \rightarrow F_a$
2. Rotate $F_a$ by angle $\theta$ around $y_a \rightarrow F_b$
3. Rotate $F_b$ by angle $\psi$ around $z_b \rightarrow F_1$

This composite rotation is represented by multiplying the basic rotation matrices:
$$R_1^0 = R_{z,\phi} R_{y,\theta} R_{z,\psi}$$

*(Note: The shorthand $c_\theta = \cos\theta$ and $s_\theta = \sin\theta$ is used throughout.)*

$$ R_1^0 = \begin{bmatrix} c_\phi & -s_\phi & 0 \\ s_\phi & c_\phi & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix} \begin{bmatrix} c_\psi & -s_\psi & 0 \\ s_\psi & c_\psi & 0 \\ 0 & 0 & 1 \end{bmatrix} $$

### Theorem on Mapping
Let $F: (\phi, \theta, \psi) \rightarrow R_1^0 \in SO(3)$. 
* **$F$ is surjective:** For any $R \in SO(3)$, there exists at least one set of ZYZ Euler angles $(\phi, \theta, \psi)$ such that $F(\phi, \theta, \psi) = R$.
* **The Catch ($F$ is not injective):** Given an $R \in SO(3)$, the angles $(\phi, \theta, \psi)$ that produce it are **not unique**.

---

## 2. The Inverse Problem for ZYZ Euler Angles

**Problem:** Given a known matrix $R \in SO(3)$, find a set of ZYZ Euler angles such that $F(\phi, \theta, \psi) = R$.

### Mathematical Prerequisite: The `atan2` Function
To solve the inverse problem robustly, we use the two-argument arctangent function:
$$\text{Atan2}: \mathbb{R}^2 - \{(0,0)\} \rightarrow (-\pi, \pi]$$
$$\text{atan2}(y, x) = \text{Arg}(x + iy)$$
This returns the principal argument of the complex number. 
* **Important:** Do *not* use $\text{atan}(y/x)$, as it cannot distinguish between the quadrants of $(x+iy)$ and $(-x-iy)$.
* **Property:** For any $\lambda > 0$, $\text{atan2}(\lambda y, \lambda x) = \text{atan2}(y, x)$.

### Solving the Inverse Problem
Let the given rotation matrix be:
$$ R = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix} = \begin{bmatrix} * & * & c_\phi s_\theta \\ * & * & s_\phi s_\theta \\ -s_\theta c_\psi & s_\theta s_\psi & c_\theta \end{bmatrix} $$

We solve this by breaking it into three cases based on $s_\theta = \sin\theta$:

#### Case 1: Assume $s_\theta > 0$
Let $\lambda := s_\theta > 0$. Using the properties of `atan2` and the elements of $R$:
* $\phi = \text{atan2}(r_{23}, r_{13})$
* $\psi = \text{atan2}(r_{32}, -r_{31})$
* Since $s_\theta > 0$, we know $s_\theta = +\sqrt{1 - c_\theta^2} = +\sqrt{1 - r_{33}^2}$. Therefore:
  $$\theta = \text{atan2}(\sqrt{1 - r_{33}^2}, r_{33})$$

#### Case 2: Assume $s_\theta < 0$
Let $\lambda := -s_\theta > 0$.
* $\phi = \text{atan2}(-r_{23}, -r_{13})$
* $\psi = \text{atan2}(-r_{32}, r_{31})$
* $\theta = \text{atan2}(-\sqrt{1 - r_{33}^2}, r_{33})$

#### Case 3: $s_\theta = 0$ (Singularity / Gimbal Lock)
This occurs if and only if $r_{13}, r_{23}, r_{31}, r_{32}$ are all $0$. In this case, there are infinitely many solutions.
* **Instructor's Mathematical Correction/Expansion:** The notes state $R = R_{z, \phi+\psi}$. To be perfectly mathematically complete:
  * If $c_\theta = 1$ (meaning $\theta = 0$), then $R = R_{z,\phi} I R_{z,\psi} = R_{z, \phi+\psi}$. You can choose any $\phi$, and set $\psi$ accordingly.
  * If $c_\theta = -1$ (meaning $\theta = \pi$), the $y$-axis rotation flips the matrix, resulting in $R = R_{z, \phi-\psi}$ (with modified signs on the $Z$ axis). 

---

## 3. Changing Coordinates of Points (Rigid Body Motion)

While vectors define directions and magnitudes, **points** describe exact locations in space. 

Let a point in Frame 1 be represented as $P^1 = \begin{bmatrix} p_x & p_y & p_z \end{bmatrix}^T \in \mathbb{R}^3$.
By definition, its physical location $P$ is relative to the origin of Frame 1 ($O_1$) and its basis vectors ($x_1, y_1, z_1$):
$$P = O_1 + p_x x_1 + p_y y_1 + p_z z_1$$

**Goal:** Find the coordinates of this point in Frame 0 ($P^0$).
$$P^0 = (O_1 + p_x x_1 + p_y y_1 + p_z z_1)^0$$
$$P^0 = O_1^0 + p_x x_1^0 + p_y y_1^0 + p_z z_1^0$$
$$P^0 = O_1^0 + R_1^0 \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix}$$

This yields the fundamental equation for **Rigid Motion**:
$$P^0 = R_1^0 P^1 + O_1^0$$
*(Note: Contrast this with the transformation of a pure vector, which only rotates: $v^0 = R_1^0 v^1$)*

---

## 4. Derivation of Elementary Rotation Matrices

The notes derive the 3x3 fundamental rotation matrices by projecting the unit basis vectors of the rotated frame ($x_1, y_1, z_1$) onto the axes of the original frame ($x_0, y_0, z_0$). 

For any rotation matrix $R_1^0$, the columns are exactly the basis vectors of Frame 1 expressed in Frame 0:
$$R_1^0 = \begin{bmatrix} | & | & | \\ x_1^0 & y_1^0 & z_1^0 \\ | & | & | \end{bmatrix}$$

### 1. Rotation around the X-axis by $\theta$ ($R_{x,\theta}$)
* The x-axis is unchanged: $x_1^0 = R_1^0 \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix}$
* The y-axis rotates towards z: $y_1^0 = R_1^0 \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ c_\theta \\ s_\theta \end{bmatrix}$
* The z-axis rotates towards -y: $z_1^0 = R_1^0 \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} = \begin{bmatrix} 0 \\ -s_\theta \\ c_\theta \end{bmatrix}$

### 2. Rotation around the Y-axis by $\theta$ ($R_{y,\theta}$)
* The y-axis is unchanged: $y_1^0 = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}$
* The x-axis rotates towards -z: $x_1^0 = \begin{bmatrix} c_\theta \\ 0 \\ -s_\theta \end{bmatrix}$
* The z-axis rotates towards x: $z_1^0 = \begin{bmatrix} s_\theta \\ 0 \\ c_\theta \end{bmatrix}$

### 3. Rotation around the Z-axis by $\theta$ ($R_{z,\theta}$)
* The z-axis is unchanged: $z_1^0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$
* The x-axis rotates towards y: $x_1^0 = \begin{bmatrix} c_\theta \\ s_\theta \\ 0 \end{bmatrix}$
* The y-axis rotates towards -x: $y_1^0 = \begin{bmatrix} -s_\theta \\ c_\theta \\ 0 \end{bmatrix}$

### Summary of Elementary Rotation Matrices
*(As a sanity check, the determinant for all rotation matrices is $1$, e.g., $\det(R_x) = c_\theta^2 + s_\theta^2 = 1$.)*

$$ R_{x,\theta} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\theta & -s_\theta \\ 0 & s_\theta & c_\theta \end{bmatrix} $$

$$ R_{y,\theta} = \begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix} $$

$$ R_{z,\theta} = \begin{bmatrix} c_\theta & -s_\theta & 0 \\ s_\theta & c_\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} $$