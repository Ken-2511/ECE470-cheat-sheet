Here is a comprehensive summary of the lecture notes. 

I have transcribed the content, organized it into logical sections, and provided the missing mathematical derivations. **Note as your TA:** I noticed a few sign errors in the student's Denavit-Hartenberg (DH) table based on the provided drawings. I have included the student's original table, followed by a corrected version with a full explanation, and completed the blank "Inverse Kinematics" section using the kinematic decoupling method mentioned in the notes.

***

# Tutorial 4: Forward and Inverse Kinematics

**Topics Covered:**
*   Forward Kinematics (DH Parameters)
*   Inverse Kinematics (using Kinematic Decoupling)
*   Example: 2012 Midterm Q2

## 1. Robot Geometry and Coordinate Frames
The problem asks for the forward and inverse kinematics of a 3-DOF robot arm. Based on the 3D, side, and top views, the robot has the following configuration:
*   **Joint 1 (Prismatic):** Moves vertically. Variable is $q_1$.
*   **Joint 2 (Prismatic):** Moves horizontally along the arm. Variable is $q_2$.
*   **Joint 3 (Revolute):** Rotates the end-effector. Variable is $q_3$.
*   This is a **PPR** (Prismatic-Prismatic-Revolute) robot.

From the home position $(q_1, q_2, q_3) = (0,0,0)$ diagrams, we can extract the base dimensions:
*   Height to horizontal link: $100$ mm
*   Lateral offset: $50$ mm
*   Base length of horizontal link: $120$ mm
*   Vertical drop to Joint 3: $80$ mm
*   Horizontal reach to End Effector: $80$ mm

---

## 2. Forward Kinematics & DH Parameters

### Original DH Table (From Notes)
*The following table is transcribed exactly as written in the notes.*

| Frame $i$ | $a_i$ <br> (@$x_i$, $O_{i-1}$ to $O_i$) | $d_i$ <br> (@$z_{i-1}$, $O_{i-1}$ to $O_i$) | $\alpha_i$ <br> (@$x_i$, $\measuredangle z_{i-1} \to z_i$) | $\theta_i$ <br> (@$z_{i-1}$, $\measuredangle x_{i-1} \to x_i$) |
| :---: | :---: | :---: | :---: | :---: |
| 1 | $50$ | $100 + q_1$ | $-\frac{\pi}{2}$ | $0$ |
| 2 | $80$ | $120 + q_2$ | $-\frac{\pi}{2}$ | $-\frac{\pi}{2}$ |
| 3 | $80$ | $0$ | $0$ | $-\frac{\pi}{2} + q_3$ |

### 🚨 TA Correction & Verification
If we multiply the transformation matrices using the student's table, Frame 2 (the revolute joint) will move *upwards* by 80 units rather than downwards as shown in the side-view drawing. 

To make the math match the physical drawing (where the joint drops down by 80), we need to correct the signs in row 2 and row 3. Assuming standard frame assignments ($x_0$ pointing out of the page, $y_0$ right, $z_0$ up):
*   To move *down* by 80, $\theta_2$ must be $+\frac{\pi}{2}$ (pointing $x_2$ up so the frame shifts downward relative to it).
*   Consequently, $\alpha_2$ and $\theta_3$ signs must adjust to maintain the correct revolute axis.

**Corrected DH Table:**
| Frame $i$ | Type | $a_i$ | $d_i$ | $\alpha_i$ | $\theta_i$ |
| :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | Prismatic | $50$ | $100 + q_1$ | $-\frac{\pi}{2}$ | $0$ |
| 2 | Prismatic | $80$ | $120 + q_2$ | **$\frac{\pi}{2}$** | **$\frac{\pi}{2}$** |
| 3 | Revolute | $80$ | $0$ | $0$ | **$\frac{\pi}{2} + q_3$** |

### Forward Kinematics Transformation Matrices
Using the standard DH transformation matrix formula:
$$ A_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\ \sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix} $$

Applying the corrected parameters yields the individual link transformations:

$$ A_1 = \begin{bmatrix} 1 & 0 & 0 & 50 \\ 0 & 0 & 1 & 0 \\ 0 & -1 & 0 & 100+q_1 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad A_2 = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 1 & 0 & 0 & 80 \\ 0 & 1 & 0 & 120+q_2 \\ 0 & 0 & 0 & 1 \end{bmatrix} $$

$$ A_3 = \begin{bmatrix} \sin q_3 & \cos q_3 & 0 & 80\sin q_3 \\ -\cos q_3 & \sin q_3 & 0 & -80\cos q_3 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} $$

The total position of the end-effector relative to the base is found by multiplying $T_3^0 = A_1 A_2 A_3$. 

---

## 3. Inverse Kinematics (Using Kinematic Decoupling)

*The second page of the notes left this section blank. Here is the complete solution.*

**Kinematic Decoupling** separates the problem into finding the position of the first few joints, and then finding the orientation of the end-effector. 

Given a desired End-Effector position $P_{ee} = [x_e, y_e, z_e]^T$ and a desired end-effector link angle $\phi$ (relative to the horizontal $y_0$ axis):

**Step 1: Determine the coordinates of Joint 3 (Wrist Center / $O_2$)**
Looking at the geometry, the end-effector is located 80 units away from Joint 3 at an angle $\phi$. We can find the position of Joint 3 ($P_2$) by working backwards from the end-effector:
$$ y_2 = y_e - 80 \cos\phi $$
$$ z_2 = z_e - 80 \sin\phi $$
*(Note: $x_e$ must always be 50, as the robot cannot move along the $x_0$ axis).*

**Step 2: Solve for $q_1$ and $q_2$ (Position)**
From our forward kinematics (evaluating $A_1 A_2 [0,0,0,1]^T$), the position of $O_2$ relative to the base is purely dependent on the prismatic joints:
*   $y_2 = 120 + q_2$
*   $z_2 = 100 + q_1 - 80 = 20 + q_1$

We equate the geometric coordinates from Step 1 to the kinematic equations to solve for the joint variables:
$$ 120 + q_2 = y_e - 80 \cos\phi \implies \mathbf{q_2 = y_e - 80 \cos\phi - 120} $$
$$ 20 + q_1 = z_e - 80 \sin\phi \implies \mathbf{q_1 = z_e - 80 \sin\phi - 20} $$

**Step 3: Solve for $q_3$ (Orientation)**
Because $q_3$ acts directly in the Y-Z plane, the joint angle is simply related to the desired physical angle $\phi$. Depending on how $\phi$ is defined relative to the home position, $q_3$ maps directly to it:
$$ \mathbf{q_3 = \phi} $$ 

*(Assuming $\phi=0$ corresponds to the end-effector pointing horizontally to the right).*