Here is a well-structured summary of the lecture notes. 

***

# Tutorial 2: Rotation Matrices

**Today's Topics:**
1. Special notations
2. Rotation matrices as a device to change coordinate representations of vectors.
3. Rotation matrices to perform rotational transformations.

## Two Uses of Rotation Matrices

1. **Change coordinates of vectors:**
   To express a vector given in Frame 1 ($v^1$) in terms of Frame 0 ($v^0$):
   $$v^0 = R_1^0 v^1$$

2. **Rotational transformation of vectors in the same frame:**
   To apply a rotation operator $L$ to a vector $v^0$ resulting in a new vector $w^0$, all within the same frame:
   $$w^0 = L(v^0) = R v^0$$

**Matrix Notation:**
A rotation matrix describing Frame 1 relative to Frame 0 is composed of the basis vectors of Frame 1 expressed in Frame 0:
$$R_1^0 = \begin{bmatrix} x_1^0 & y_1^0 & z_1^0 \end{bmatrix}$$

---

## Example 1: Coordinate Transformations

We are given a system with three coordinate frames (Frame 0, Frame 1, Frame 2). 

**Frame 1 (relative to Frame 0):**
*   $y_1 = z_0$
*   The $z_1$-$x_1$ plane is parallel (//) to the $x_0$-$y_0$ plane.
*   The angle between $z_1$ and $x_0$ is $\theta$.

**Frame 2 (relative to Frame 1):**
*   $z_2 = x_1$
*   $x_2 = z_1$
*   $y_2 = -y_1$

Given a vector expressed in Frame 2:
$$v^2 = \begin{bmatrix} 1/2 \\ -1/2 \\ 1/2 \end{bmatrix}$$

**Find:** (a) $v^1$, and (b) $v^0$.

### Part (a) Find $v^1$
Using the transformation equation $v^1 = R_2^1 v^2$. First, we must construct the rotation matrix $R_2^1$.
$$R_2^1 = \begin{bmatrix} x_2^1 & y_2^1 & z_2^1 \end{bmatrix}$$

By looking at the basis vectors of Frame 2 in terms of Frame 1:
*   $x_2^1 = z_1^1 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$
*   $y_2^1 = -y_1^1 = \begin{bmatrix} 0 \\ -1 \\ 0 \end{bmatrix}$
*   $z_2^1 = x_1^1 = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix}$

Therefore, the rotation matrix is:
$$R_2^1 = \begin{bmatrix} 0 & 0 & 1 \\ 0 & -1 & 0 \\ 1 & 0 & 0 \end{bmatrix}$$

Now, multiply $R_2^1$ by $v^2$:
$$v^1 = R_2^1 v^2 = \begin{bmatrix} 0 & 0 & 1 \\ 0 & -1 & 0 \\ 1 & 0 & 0 \end{bmatrix} \begin{bmatrix} 1/2 \\ -1/2 \\ 1/2 \end{bmatrix} = \begin{bmatrix} 1/2 \\ 1/2 \\ 1/2 \end{bmatrix}$$

### Part (b) Find $v^0$
The formula is:
$$v^0 = R_1^0 v^1$$

> **Instructor / Assistant Note (Filling in the Gap):** 
> The original notes stop here and leave $R_1^0$ uncalculated. To make this complete, we can construct $R_1^0$ from the geometric description provided:
> 1. $y_1 = z_0 \implies y_1^0 = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}^T$
> 2. $z_1$ is in the $x_0$-$y_0$ plane at an angle $\theta$ from $x_0$. Assuming a standard positive (counterclockwise) rotation: $z_1^0 = \begin{bmatrix} \cos\theta & \sin\theta & 0 \end{bmatrix}^T$
> 3. By the right-hand rule, $x_1 = y_1 \times z_1$. Doing the cross product in Frame 0 yields: $x_1^0 = \begin{bmatrix} -\sin\theta & \cos\theta & 0 \end{bmatrix}^T$.
> 
> Thus, $R_1^0 = \begin{bmatrix} -\sin\theta & 0 & \cos\theta \\ \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \end{bmatrix}$. 
> You would then calculate $v^0 = R_1^0 \begin{bmatrix} 1/2 \\ 1/2 \\ 1/2 \end{bmatrix}$ to finish the problem.

---

## Example 2: Successive Rotations in a Fixed Frame

Given an initial vector $u^0 = \begin{bmatrix} 1 \\ 1 \\ 0 \end{bmatrix}$.
A final vector $w$ is obtained by the following sequence:
1. Rotate $u$ about the $x_0$ axis by $\frac{\pi}{2}$ to get $v$.
2. Rotate $v$ about the $z_0$ axis by $\frac{\pi}{3}$ to get $w$.

**Find $w^0$.**

Since all rotations are performed relative to the fixed Frame 0, we pre-multiply the rotation matrices:
$$v^0 = R_{x, \frac{\pi}{2}} u^0$$
$$w^0 = R_{z, \frac{\pi}{3}} v^0 = R_{z, \frac{\pi}{3}} R_{x, \frac{\pi}{2}} u^0$$

Calculating the result:
$$w^0 = R_{z, \frac{\pi}{3}} R_{x, \frac{\pi}{2}} u^0 = \begin{bmatrix} 1/2 \\ \sqrt{3}/2 \\ 1 \end{bmatrix}$$

---

## Example 3: Successive Rotations involving Local Frames

**Setup:**
*   **Frame 0** is the base frame.
*   **Frame 1** is obtained by rotating Frame 0 by $\frac{\pi}{2}$ about the $x_0$ axis. 
    *(This means $R_1^0 = R_{x, \frac{\pi}{2}}$ and $R_0^1 = (R_1^0)^T = R_{x, -\frac{\pi}{2}}$)*

Vector $w$ is obtained by:
1. Same as before $\rightarrow$ Vector $v$ is obtained by rotating $u$ about $x_0$ by $\frac{\pi}{2}$.
   $$(v^0 = R_{x, \frac{\pi}{2}} u^0)$$
2. Vector $v$ is then rotated about the **local $z_1$ axis** by $\frac{\pi}{3} \rightarrow w$.

**Find $w^0$.**

**Derivation:**
Because the second rotation happens about a local axis ($z_1$), we apply the rotation operator in Frame 1:
$$w^1 = R_{z, \frac{\pi}{3}} v^1$$

We also know the coordinate transformation rule back to Frame 0:
$$w^0 = R_1^0 w^1$$

Substitute $w^1$ into the equation:
$$w^0 = R_1^0 \left( R_{z, \frac{\pi}{3}} v^1 \right)$$

Next, substitute the relationship for $v^1$ (which is $v^1 = R_0^1 v^0$):
$$w^0 = R_1^0 R_{z, \frac{\pi}{3}} R_0^1 v^0$$

Finally, substitute the definition of $v^0$ ($v^0 = R_{x, \frac{\pi}{2}} u^0$):
$$w^0 = R_1^0 R_{z, \frac{\pi}{3}} \underbrace{R_0^1 R_{x, \frac{\pi}{2}}}_{\text{cancels}} u^0$$

**Why does it cancel?** 
Because Frame 1 is defined by a rotation of $\frac{\pi}{2}$ about $x_0$, $R_1^0 = R_{x, \frac{\pi}{2}}$. Therefore, $R_0^1$ is the inverse of $R_{x, \frac{\pi}{2}}$. Multiplying them together yields the identity matrix.

The simplified equation is:
$$w^0 = R_1^0 R_{z, \frac{\pi}{3}} u^0$$

**Final Calculation:**
Substitute the actual matrices into the simplified equation ($R_1^0$ is $R_{x, \frac{\pi}{2}}$):

$$
w^0 = 
\underbrace{\begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{bmatrix}}_{R_{x, \frac{\pi}{2}}}
\underbrace{\begin{bmatrix} 1/2 & -\sqrt{3}/2 & 0 \\ \sqrt{3}/2 & 1/2 & 0 \\ 0 & 0 & 1 \end{bmatrix}}_{R_{z, \frac{\pi}{3}}}
\underbrace{\begin{bmatrix} 1 \\ 1 \\ 0 \end{bmatrix}}_{u^0}
= \begin{bmatrix} \frac{1}{2} - \frac{\sqrt{3}}{2} \\ 0 \\ \frac{1}{2} + \frac{\sqrt{3}}{2} \end{bmatrix}
$$