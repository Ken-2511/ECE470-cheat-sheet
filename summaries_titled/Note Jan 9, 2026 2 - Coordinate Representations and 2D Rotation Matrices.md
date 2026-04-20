Here is a structured summary of the lecture notes. The content has been organized into logical sections with LaTeX formatting for clarity. 

Where the notes imply a concept but don't explicitly write the final equation, I have added brief "TA Notes" to complete the thought and ensure the summary is self-contained.

***

# ECE470 Robotics: Lecture Notes Summary

## 1. Geometric Points and Vectors
Before defining coordinate systems, we differentiate between abstract points and vectors:
*   **Geometric Point:** An abstract quantity indicating a position in space.
*   **Geometric Vector:** A quantity that indicates a direction with a specific magnitude.
    *   *Note:* Geometric vectors have "floating tails." This means a vector is defined solely by its length and direction, regardless of where it starts in space (it is translation-invariant).

## 2. Coordinate Representation of Points and Vectors

To express points and vectors mathematically, we fix a reference frame $i$, defined by an origin $o_i$ and a set of orthogonal basis axes $x_i, y_i, z_i$.

### Representing Vectors
Let $v$ be an abstract geometric vector. We can express $v$ as a linear combination of the basis vectors of frame $i$:
$$v = v_x x_i + v_y y_i + v_z z_i$$

*   The set $\{x_i, y_i, z_i\}$ forms a basis in the vector space.
*   The **coordinate representation** of $v$ with respect to (w.r.t.) frame $i$ is defined as the $3 \times 1$ real column vector $v^i$:
$$v^i = \begin{bmatrix} v_x \\ v_y \\ v_z \end{bmatrix}$$

### Representing Points
Let $p$ be an abstract geometric point. Unlike vectors, points are tied to an origin. We define $p$ relative to the origin $o_i$ of frame $i$:
$$p = o_i + p_x x_i + p_y y_i + p_z z_i$$

*   We define the coordinate representation of point $p$ in frame $i$ to be a $3 \times 1$ real vector. 
*   *(TA Note for completeness: Just like the vector $v$, this coordinate representation is written as $p^i = \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix}$).*

### Multiple Frames (Example)
In robotics (like the articulated arm sketched in the notes), a robot will have multiple joints and links. We assign different coordinate frames to these parts (e.g., Frame 0 at the base, Frame 1 at the first joint, Frame 2 at the second joint). This motivates the need to translate and rotate representations between frames.

---

## 3. Rotations on the Plane (2D)

**Motivation:** We want to understand how to indicate a rotation of frame $i$ w.r.t. frame $j$. 

Consider two coordinate frames (Frame 0 and Frame 1) that share the same origin on a 2D plane. Frame 1 is rotated counterclockwise by an angle $\theta$ relative to Frame 0.

### The Rotation Matrix
We define the rotation matrix $R^0_1$, which represents the rotation of Frame 1 w.r.t. Frame 0. It is constructed by projecting the basis vectors of Frame 1 ($x_1, y_1$) onto the basis vectors of Frame 0 ($x_0, y_0$).

$$R^0_1 := \begin{bmatrix} x^0_1 & \vdots & y^0_1 \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$$

This matrix is built column-by-column:
1.  **First column ($x^0_1$):** The coordinates of Frame 1's x-axis ($x_1$) expressed in Frame 0.
    $$x^0_1 = \begin{bmatrix} a \\ b \end{bmatrix} = \begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix}$$
2.  **Second column ($y^0_1$):** The coordinates of Frame 1's y-axis ($y_1$) expressed in Frame 0.
    $$y^0_1 = \begin{bmatrix} c \\ d \end{bmatrix} = \begin{bmatrix} -\sin\theta \\ \cos\theta \end{bmatrix}$$

### Characterizing Rotation using the Dot Product
We can systematically construct this rotation matrix using the dot product. The dot product computes the length of the projection of one unit vector onto another. 

Therefore, the exact components of $R^0_1$ are the dot products of the respective basis vectors:
$$R^0_1 = \begin{bmatrix} x_1 \cdot x_0 & y_1 \cdot x_0 \\ x_1 \cdot y_0 & y_1 \cdot y_0 \end{bmatrix}$$

### Transpose Property (Inverse Rotation)
If we want the reverse—the rotation of Frame 0 w.r.t. Frame 1 ($R^1_0$)—we construct the matrix using the columns $x^1_0$ and $y^1_0$:

$$R^1_0 = \begin{bmatrix} x^1_0 & \vdots & y^1_0 \end{bmatrix} = \begin{bmatrix} x_0 \cdot x_1 & y_0 \cdot x_1 \\ x_0 \cdot y_1 & y_0 \cdot y_1 \end{bmatrix}$$

Because the dot product is commutative ($a \cdot b = b \cdot a$), we can easily see that this new matrix is simply the transpose of the original rotation matrix:
$$R^1_0 = (R^0_1)^T$$

*(TA Note: This is a fundamental property of rotation matrices. Because basis vectors are orthogonal unit vectors, a rotation matrix is an Orthogonal Matrix. This means its inverse is equal to its transpose: $(R^0_1)^{-1} = (R^0_1)^T = R^1_0$)*.