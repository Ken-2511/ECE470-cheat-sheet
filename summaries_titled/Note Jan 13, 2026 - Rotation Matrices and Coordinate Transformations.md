Here is a structured summary of your lecture notes for ECE470. I have organized the content, formatted the math using LaTeX, and added a few "TA Notes" to clarify missing steps and notation.

---

# Robotics Lecture Notes Summary: Rotation Matrices

## Two Different Applications of Rotation Matrices
1. Use them to **change coordinates of vectors** (covered today).
2. Use them as **"rotators"**, which are operators that rotate vectors within a given frame (to be covered tomorrow).

---

## 1. Changing Coordinates of Vectors

**Motivation:**
Suppose we have a vector defined in Frame 1 ($v^1$) and we want to find its relationship to Frame 0 ($v^0$). 
Given:
$$v^1 = \begin{bmatrix} v_x \\ v_y \\ v_z \end{bmatrix} \in \mathbb{R}^3$$
Find: $v^0 \in \mathbb{R}^3$

**Derivation:**
A vector $v$ can be expressed as a linear combination of the unit basis vectors of Frame 1 ($x_1, y_1, z_1$):
$$v = v_x x_1 + v_y y_1 + v_z z_1$$

To express this vector in Frame 0, we project the basis vectors of Frame 1 onto Frame 0:
$$v^0 = (v_x x_1 + v_y y_1 + v_z z_1)^0 \in \mathbb{R}^3$$
$$v^0 = v_x x_1^0 + v_y y_1^0 + v_z z_1^0$$

*Note: $v_x, v_y, v_z$ are scalar components, while $x_1^0, y_1^0, z_1^0$ are $3 \times 1$ vectors representing the axes of Frame 1 relative to Frame 0.*

We can rewrite this linear combination as a matrix-vector multiplication:
$$v^0 = \underbrace{\begin{bmatrix} x_1^0 & y_1^0 & z_1^0 \end{bmatrix}}_{R_1^0} \underbrace{\begin{bmatrix} v_x \\ v_y \\ v_z \end{bmatrix}}_{v^1}$$

**General Rule:**
In general, to change the coordinates of a vector from Frame $j$ to Frame $i$, you multiply by the rotation matrix $R_j^i$:
$$v^i = R_j^i v^j$$

---

## 2. Orthogonal Matrices & The Special Orthogonal Group $SO(3)$

### The Orthogonal Group $O(3)$
A matrix $R \in \mathbb{R}^{3 \times 3}$ such that $R^T R = I_3$ is said to be **orthogonal**. The set of all $3 \times 3$ orthogonal matrices is called $O(3)$.

> **TA Note - Filling in the Math Gap:** 
> The notes skip a few steps when proving the determinant of an orthogonal matrix. Here is the complete derivation showing why the determinant must be $\pm 1$:
> 1. Start with the property: $R^T R = I_3$
> 2. Take the determinant of both sides: $\det(R^T R) = \det(I_3)$
> 3. Use the product rule $\det(A \cdot B) = \det(A) \cdot \det(B)$: $\det(R^T) \cdot \det(R) = 1$
> 4. Since the determinant of a transpose is equal to the determinant of the original matrix ($\det(R^T) = \det(R)$), we get: $\det(R)^2 = 1$
> 5. Therefore: $\det(R) = \pm 1$

**Frame Representation and Handedness:**
If $R \in O(3)$, we can write it as a collection of column vectors: $R = \begin{bmatrix} R^1 & R^2 & R^3 \end{bmatrix}$. 
These vectors represent an orthonormal frame with an unspecified origin.
*   If $\det(R) = 1$, then the frame generated is **right-handed**.
*   If $\det(R) = -1$, then the frame generated is **left-handed**.

### The Special Orthogonal Group $SO(3)$
In robotics, we are specifically interested in right-handed coordinate frames. This subset of $O(3)$ is called the Special Orthogonal Group, $SO(3)$:
$$SO(3) = \{ R \in \mathbb{R}^{3 \times 3} \mid R^T R = I_3, \det(R) = 1 \}$$
**All of our rotation matrices $R_i^j$ belong to $SO(3)$.**

---

## 3. Composition of Rotations

**Motivation:**
If you know how to get from Frame 2 to Frame 1, and Frame 1 to Frame 0, how do you get from Frame 2 to Frame 0?

$$v^2 = \begin{bmatrix} v_x \\ v_y \\ v_z \end{bmatrix} \implies v^1 = R_2^1 v^2$$
Substituting $v^1$ into the equation for Frame 0:
$$v^0 = R_1^0 (R_2^1 v^2)$$
$$v^0 = (R_1^0 R_2^1) v^2$$
$$v^0 = R_2^0 v^2$$
This shows that successive coordinate transformations can be composed via matrix multiplication.

---

## 4. Elementary Rotations

These are the standard rotation matrices for rotating a frame by an angle $\theta$ about the principal axes ($x, y,$ or $z$). All of these matrices have a determinant of $1$ ($\det = 1$).

> **TA Note - Notation Clarification:** 
> In the matrices below, the notes use shorthand trigonometric notation. 
> Please note that $c_\theta = \cos(\theta)$ and $s_\theta = \sin(\theta)$.

**Rotation about the X-axis ($x_0$):**
$$R_1^0 := R_{x\theta} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\theta & -s_\theta \\ 0 & s_\theta & c_\theta \end{bmatrix}$$

**Rotation about the Y-axis:**
$$R_{y\theta} = \begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix}$$

**Rotation about the Z-axis:**
$$R_{z\theta} = \begin{bmatrix} c_\theta & -s_\theta & 0 \\ s_\theta & c_\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}$$