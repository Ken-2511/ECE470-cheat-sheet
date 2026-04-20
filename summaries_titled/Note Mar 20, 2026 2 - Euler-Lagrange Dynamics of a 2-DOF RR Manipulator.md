Here is a structured summary of the lecture notes on robot dynamics (ECE470). 

*Note: As an ECE470 TA, I have reviewed the math carefully. I have filled in some missing intermediate steps (like explicitly defining the inertia matrix $D(q)$) to make the derivation complete. **I also found two errors in the calculation of the Coriolis matrix $C$ in the original notes, which I have pointed out and corrected below.***

---

# ECE470 Robotics: Robot Dynamics Summary

## 1. General Dynamics Formulations

The dynamics of a robot manipulator can be derived using energy methods.
*   **Kinetic Energy:** $K(q, \dot{q}) = \frac{1}{2} \dot{q}^T D(q) \dot{q}$
    *(where $D(q)$ is the symmetric, positive-definite Inertia Matrix)*
*   **Potential Energy:** $P(q) =$ Sum of the gravitational potential of each link.

There are two primary approaches to formulating the equations of motion:

**Approach 1: Euler-Lagrange Formulation**
Using the Lagrangian $\mathcal{L}(q, \dot{q}) = K(q, \dot{q}) - P(q)$:
$$ \frac{d}{dt} \nabla_{\dot{q}} \mathcal{L}(q, \dot{q}) - \nabla_q \mathcal{L}(q, \dot{q}) = \tau $$

**Approach 2: Standard Equations of Motion Form**
Expanding the Euler-Lagrange equations yields the standard matrix form:
$$ D(q)\ddot{q} + C(q, \dot{q})\dot{q} + \nabla_q P(q) = \tau $$
*(where $C(q, \dot{q})$ is the Coriolis/Centrifugal matrix and $\nabla_q P(q)$ is the Gravity vector).*

---

## 2. Example: Planar 2-DOF RR Manipulator

The notes demonstrate Approach 2 for a 2-link planar revolute-revolute (RR) robot. Let $S_i = \sin(q_i)$, $C_i = \cos(q_i)$, $S_{12} = \sin(q_1+q_2)$, and $C_{12} = \cos(q_1+q_2)$.

### Step A: Kinematics (Positions and Velocities of Centers of Mass)
From the blackboard, the positions ($r_{ci}^0$) and linear velocities ($\dot{r}_{ci}^0$) of the center of mass for each link are:
*   **Link 1:**
    $$ r_{c1}^0 = \begin{bmatrix} l_{c1} \cos(q_1) \\ l_{c1} \sin(q_1) \\ 0 \end{bmatrix}, \quad \dot{r}_{c1}^0 = \begin{bmatrix} -l_{c1} \sin(q_1) \dot{q}_1 \\ l_{c1} \cos(q_1) \dot{q}_1 \\ 0 \end{bmatrix} $$
*   **Link 2:**
    $$ r_{c2}^0 = \begin{bmatrix} l_1 \cos(q_1) + l_{c2} \cos(q_1+q_2) \\ l_1 \sin(q_1) + l_{c2} \sin(q_1+q_2) \\ 0 \end{bmatrix}, \quad \dot{r}_{c2}^0 = \begin{bmatrix} -l_1 \sin(q_1) \dot{q}_1 - l_{c2} \sin(q_1+q_2) (\dot{q}_1+\dot{q}_2) \\ l_1 \cos(q_1) \dot{q}_1 + l_{c2} \cos(q_1+q_2) (\dot{q}_1+\dot{q}_2) \\ 0 \end{bmatrix} $$

The angular velocities are:
$$ \omega_1^0 = \begin{bmatrix} 0 \\ 0 \\ \dot{q}_1 \end{bmatrix}, \quad \omega_2^0 = \begin{bmatrix} 0 \\ 0 \\ \dot{q}_1 + \dot{q}_2 \end{bmatrix} $$

### Step B: Kinetic Energy & Inertia Matrix $D(q)$
The total kinetic energy is the sum of translational and rotational kinetic energy for both links:
$$ K(q, \dot{q}) = \frac{1}{2} m_1 \|\dot{r}_{c1}^0\|^2 + \frac{1}{2}\omega_1^T I_1 \omega_1 + \frac{1}{2} m_2 \|\dot{r}_{c2}^0\|^2 + \frac{1}{2}\omega_2^T I_2 \omega_2 $$

Expanding and simplifying the squared velocity magnitudes (using $\cos(A-B)$ identities):
$$ \|\dot{r}_{c1}^0\|^2 = l_{c1}^2 S_{q_1}^2 \dot{q}_1^2 + l_{c1}^2 C_{q_1}^2 \dot{q}_1^2 = l_{c1}^2 \dot{q}_1^2 $$
$$ \|\dot{r}_{c2}^0\|^2 = l_1^2 \dot{q}_1^2 + l_{c2}^2(\dot{q}_1+\dot{q}_2)^2 + 2l_1 l_{c2} \cos(q_2) \dot{q}_1(\dot{q}_1+\dot{q}_2) $$

Substituting these into the full $K$ equation shown on the blackboard allows us to extract the Inertia Matrix $D(q)$ by matching terms to $\frac{1}{2} \begin{bmatrix} \dot{q}_1 & \dot{q}_2 \end{bmatrix} \begin{bmatrix} D_{11} & D_{12} \\ D_{21} & D_{22} \end{bmatrix} \begin{bmatrix} \dot{q}_1 \\ \dot{q}_2 \end{bmatrix}$:

*(Note: The notes skip explicitly writing the $D$ matrix out, but it is necessary for the next step. Here it is derived from the kinetic energy equation):*
*   $D_{11} = m_1 l_{c1}^2 + I_{z1} + m_2 (l_1^2 + l_{c2}^2 + 2 l_1 l_{c2} \cos(q_2)) + I_{z2}$
*   $D_{12} = D_{21} = m_2 (l_{c2}^2 + l_1 l_{c2} \cos(q_2)) + I_{z2}$
*   $D_{22} = m_2 l_{c2}^2 + I_{z2}$

### Step C: Coriolis Matrix $C(q, \dot{q})$
The elements of the Coriolis matrix $C_{kj}$ are computed using Christoffel symbols of the first kind:
$$ C_{kj} = \sum_{i=1}^n C_{ijk}(q) \dot{q}_i \quad \text{where} \quad C_{ijk} = \frac{1}{2} \left( \frac{\partial D_{kj}}{\partial q_i} + \frac{\partial D_{ki}}{\partial q_j} - \frac{\partial D_{ij}}{\partial q_k} \right) $$
*Important property:* $C_{ijk} = C_{jik}$ (symmetric in the first two indices).

First, define the substitution variable **$C = -m_2 l_1 l_{c2} \sin(q_2)$**. 
Taking the partial derivatives of the $D(q)$ elements derived above:
*   $\frac{\partial D_{11}}{\partial q_1} = 0$, $\quad \frac{\partial D_{11}}{\partial q_2} = -2m_2 l_1 l_{c2} \sin(q_2) = 2C$
*   $\frac{\partial D_{12}}{\partial q_1} = 0$, $\quad \frac{\partial D_{12}}{\partial q_2} = -m_2 l_1 l_{c2} \sin(q_2) = C$
*   $\frac{\partial D_{22}}{\partial q_1} = 0$, $\quad \frac{\partial D_{22}}{\partial q_2} = 0$

**Calculating Christoffel Symbols ($C_{ijk}$)**
> ⚠️ **Correction to Notes:** The handwritten notes incorrectly write $C_{112} = C$. If we look at the blackboard inset image, the professor correctly calculated it as $-c$. Applying the formula $C_{112} = \frac{1}{2}(0 + 0 - 2C)$ yields $-C$. I have corrected this and filled in the blanks left in the notes below:

*   $C_{111} = \frac{1}{2}(0 + 0 - 0) = 0$
*   **$C_{112} = \frac{1}{2}(\frac{\partial D_{21}}{\partial q_1} + \frac{\partial D_{21}}{\partial q_1} - \frac{\partial D_{11}}{\partial q_2}) = \frac{1}{2}(0 + 0 - 2C) = -C$** *(Corrected)*
*   $C_{121} = C_{211} = \frac{1}{2}(\frac{\partial D_{12}}{\partial q_1} + \frac{\partial D_{11}}{\partial q_2} - \frac{\partial D_{12}}{\partial q_1}) = \frac{1}{2}(2C) = C$
*   $C_{221} = \frac{1}{2}(\frac{\partial D_{12}}{\partial q_2} + \frac{\partial D_{12}}{\partial q_2} - \frac{\partial D_{22}}{\partial q_1}) = \frac{1}{2}(C + C - 0) = C$
*   $C_{122} = C_{212} = \frac{1}{2}(\frac{\partial D_{22}}{\partial q_1} + \frac{\partial D_{21}}{\partial q_2} - \frac{\partial D_{12}}{\partial q_2}) = \frac{1}{2}(0 + C - C) = 0$
*   $C_{222} = \frac{1}{2}(0 + 0 - 0) = 0$

**Constructing the Matrix $C(q, \dot{q})$**
> ⚠️ **Correction to Notes:** The notes contain a formula error for $C_{12}$. The note states $C_{12} = \sum C_{i21}\dot{q}_i$. Based on the definition $C_{kj} = \sum C_{ijk}\dot{q}_i$, it must be $C_{12} = \sum C_{i12}\dot{q}_i$. Because of this error, the notes left the final result for $C_{12}$ blank. I have provided the correct derivations for all four elements:

*   $C_{11} = \sum_{i=1}^2 C_{i11}\dot{q}_i = C_{111}\dot{q}_1 + C_{211}\dot{q}_2 = 0 + C\dot{q}_2 = \mathbf{C\dot{q}_2}$
*   **$C_{12} = \sum_{i=1}^2 C_{i12}\dot{q}_i = C_{112}\dot{q}_1 + C_{212}\dot{q}_2 = -C\dot{q}_1 + 0 = \mathbf{-C\dot{q}_1}$** *(Corrected)*
*   $C_{21} = \sum_{i=1}^2 C_{i21}\dot{q}_i = C_{121}\dot{q}_1 + C_{221}\dot{q}_2 = C\dot{q}_1 + C\dot{q}_2 = \mathbf{C(\dot{q}_1 + \dot{q}_2)}$
*   $C_{22} = \sum_{i=1}^2 C_{i22}\dot{q}_i = C_{122}\dot{q}_1 + C_{222}\dot{q}_2 = 0 + 0 = \mathbf{0}$

Resulting Coriolis Matrix:
$$ C(q, \dot{q}) = \begin{bmatrix} C \dot{q}_2 & -C \dot{q}_1 \\ C(\dot{q}_1 + \dot{q}_2) & 0 \end{bmatrix} $$

### Step D: Potential Energy & Gravity Vector $\nabla_q P(q)$
The total potential energy is the sum of the potential energies of link 1 and link 2 (based on the $y$-coordinates of their centers of mass):
$$ P(q) = m_1 g l_{c1} \sin(q_1) + m_2 g (l_1 \sin(q_1) + l_{c2} \sin(q_1+q_2)) $$

Taking the gradient with respect to $q$ yields the gravity vector:
$$ \nabla_q P(q) = \begin{bmatrix} \frac{\partial P}{\partial q_1} \\ \frac{\partial P}{\partial q_2} \end{bmatrix} = \begin{bmatrix} (m_1 l_{c1} + m_2 l_1) g \cos(q_1) + m_2 l_{c2} g \cos(q_1+q_2) \\ m_2 l_{c2} g \cos(q_1+q_2) \end{bmatrix} $$