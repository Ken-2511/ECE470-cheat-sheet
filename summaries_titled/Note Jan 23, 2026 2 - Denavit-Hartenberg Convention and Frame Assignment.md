Here is a complete, well-structured summary of the lecture notes. 

***

# Robotics Tutorial (GB 248): Denavit-Hartenberg (DH) Convention

## 1. DH Frame Assignment Convention
The notes outline the standard algorithm for assigning reference frames to a robotic manipulator using the Denavit-Hartenberg (DH) convention. 

Let $l_i$ be the line representing the **joint axis of joint $i+1$**.

**Step-by-Step Assignment:**
1. **(0) Identify lines:** Identify all joint axis lines $l_0, l_1, \dots, l_{n-1}$.
2. **(1) Base Frame ($i=0$):** Pick the origin $O_0$ arbitrarily on $l_0$. Choose $X_0 \perp Z_0$ arbitrarily. *(Note: $Z_0$ aligns with $l_0$)*.
3. **(2) End-Effector Frame ($i=n$):** Pick $l_n$, with $Z_n \parallel l_n$ arbitrarily (often aligned with the tool approach direction).
4. **(3) Intermediate Origins ($O_i$):** Determine the common normal $n_i$ that is perpendicular to both $l_{i-1}$ and $l_i$. Set the origin $O_i$ at the intersection of the common normal and the current joint axis: 
   $$O_i = n_i \cap l_i$$
5. **(4) X-axes ($X_i$):** Pick $X_i$ to be parallel to the common normal $n_i$:
   $$X_i \parallel n_i$$

---

## 2. DH Parameters

To represent the kinematics of the robot, we form an $n \times 4$ table called the **DH Table**, consisting of four parameters for each link $i$: $a_i, \alpha_i, d_i, \theta_i$.

### Basic Features & Definitions
*   **Vector $O_i - O_{i-1}$**: This vector lies completely in the plane containing the lines $l_{i-1}$ and $n_i$.
*   **$\theta_i$ (Joint Angle):** The angle from $X_{i-1}$ to $X_i$ measured about $Z_{i-1}$.
*   **$d_i$ (Link Offset):** The distance from $X_{i-1}$ to $X_i$ measured along $Z_{i-1}$.

> ⚠️ **Correction / Clarification on Notes:** 
> The handwritten notes contain a confusing shorthand: "$Z_{i-1}, Z_i \perp X_i \rightarrow d_i$". This is technically incorrect based on standard DH convention. The correct parameters related to the $X_i$ axis are:
> *   **$a_i$ (Link Length):** The distance from $Z_{i-1}$ to $Z_i$ measured along $X_i$.
> *   **$\alpha_i$ (Link Twist):** The angle from $Z_{i-1}$ to $Z_i$ measured about $X_i$.

### Axis Exceptions
When assigning frames, special geometric cases (exceptions) often arise between adjacent joint axes $l_{i-1}$ and $l_i$:

1. **Exception 1: Parallel Axes ($l_{i-1} \parallel l_i$)**
   * There are infinite common normals $n_i$. 
   * *Resolution:* Pick the normal line $n_i$ that passes through the previous origin $O_{i-1}$ to force $d_i = 0$. Set $X_i \parallel n_i$.
2. **Exception 2: Intersecting Axes ($l_{i-1} \cap l_i \neq \emptyset$)**
   * The axes intersect at a single point, so the distance between them is zero ($a_i = 0$).
   * *Resolution:* Set the origin at the intersection point: $O_i = l_{i-1} \cap l_i$. Set the X-axis perpendicular to both Z-axes using the cross product: 
     $$X_i = \pm (Z_{i-1} \times Z_i)$$
3. **Exception 3: Collinear Axes ($l_{i-1} = l_i$)** *(Referenced in Example)*
   * The axes represent the same line. 
   * *Resolution:* The origin $O_i$ and $X_i$ axis can be chosen arbitrarily (often chosen to align with the previous frame to make $d_i = 0$ and $\theta_i = 0$ when in the zero-position).

---

## 3. Example: 6-DOF Manipulator with Spherical Wrist

The second part of the notes applies these rules to a 6-axis manipulator. The arm features an elbow joint and a **Spherical Wrist** (where axes 4, 5, and 6 intersect at a common point).

### Frame Assignment Logic & Exceptions Applied:
*   **Joint 1 & 2 ($l_0 \cap l_1 \neq \emptyset$):** Axes intersect. Trigger **Exception 2**. ($a_1 = 0$)
*   **Joint 2 & 3 ($l_1 \parallel l_2$):** Axes are parallel. Trigger **Exception 1**. ($X_2$ points from $l_1$ to $l_2$).
*   **Joint 3 & 4 ($l_2 \cap l_3 \neq \emptyset$):** Axes intersect. Trigger **Exception 2**. 
    *   $X_3 = Z_2 \times Z_3$
*   **Joint 4 & 5 ($l_3 \cap l_4 \neq \emptyset$):** Axes intersect (Spherical Wrist). Trigger **Exception 2**.
    *   $X_4 = -Z_3 \times Z_4$ *(Note: The negative sign is a deliberate choice to yield $\alpha_4 = -\pi/2$)*
*   **Joint 5 & 6 ($l_4 \cap l_5 \neq \emptyset$):** Axes intersect (Spherical Wrist). Trigger **Exception 2**.
    *   $X_5 = Z_4 \times Z_5$
*   **End Effector ($l_5 = l_6$):** Axes are collinear. Trigger **Exception 3**.

### Completed DH Table
Based on the $\alpha_i$ column provided in the handwritten table and the distances ($a_i, d_i$) labeled explicitly on the diagram, we can reconstruct the complete, self-contained DH table for this robot. 

*(Note: $\theta_i$ values are the variable joint angles, denoted here as $\theta_i^*$)*

| Link $i$ | $a_i$ (Length) | $\alpha_i$ (Twist) | $d_i$ (Offset) | $\theta_i$ (Angle) | Notes from Diagram |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | $0$ | $\frac{\pi}{2}$ | $d_1$ | $\theta_1^*$ | Intersecting axes ($a_1=0$) |
| **2** | $a_2$ | $0$ | $0$ | $\theta_2^*$ | Parallel axes ($l_1 \parallel l_2$) |
| **3** | $0$ | $\frac{\pi}{2}$ | $0$ | $\theta_3^*$ | Intersecting ($a_3=0, O_2=O_3$) |
| **4** | $0$ | $-\frac{\pi}{2}$ | $d_4$ | $\theta_4^*$ | Wrist entry ($a_4=0$) |
| **5** | $0$ | $\frac{\pi}{2}$ | $0$ | $\theta_5^*$ | Wrist center ($a_5=0, d_5=0, O_4=O_5$) |
| **6** | $0$ | $0$ | $d_6$ | $\theta_6^*$ | To end-effector ($a_6=0$) |