Here is a comprehensive and structured summary of the provided lecture notes on the dynamics of a cart-pole (pendulum on a cart) system. 

I have carefully checked the derivations, corrected a few notational typos from the handwritten notes, and filled in the missing steps (like the final equations of motion and the Christoffel matrix) to make this summary complete and self-contained.

---

# Dynamics of a Cart-Pole System

## System Definition
*   **$q_1$**: Horizontal displacement of the cart.
*   **$q_2$**: Angular displacement of the rod (pendulum) measured from the horizontal.
*   **Cart mass**: $M_1$
*   **Rod mass**: $M_2$
*   **$l_c$**: Distance from the joint to the center of mass of the rod.
*   **$a_1, a_2$**: Vertical offsets defining the height of the joint.

## General Energy Equation
The general kinetic energy $T$ for an $n$-link robotic system is given by the sum of translational and rotational kinetic energies:
$$T = \sum_{i=1}^n \left( \frac{1}{2} m_i \|\dot{r}_{ci}^0\|^2 + \frac{1}{2} (\omega_i^0)^T I_i \omega_i^0 \right)$$
*(Note: The handwritten notes contain a slight notational typo in the rotational term, writing it as $(\omega_i)^2 I_i \omega_i^\circ$. The standard matrix form is used above).*

---

## Approach 1: Euler-Lagrange using Energy Equations

The first approach involves finding the kinetic and potential energies, constructing the Lagrangian $\mathcal{L}(q, \dot{q})$, and applying the Euler-Lagrange equations.

### 1. Kinematics (Positions and Velocities)
Define the position vectors of the center of masses ($r_{c1}^0$ for the cart, $r_{c2}^0$ for the rod):
$$r_{c1}^0 = \begin{bmatrix} q_1 \\ a_1 \\ 0 \end{bmatrix} \implies \dot{r}_{c1}^0 = \begin{bmatrix} \dot{q}_1 \\ 0 \\ 0 \end{bmatrix}$$
$$r_{c2}^0 = \begin{bmatrix} q_1 + d_2 + l_c \cos(q_2) \\ a_1 + a_2 + l_c \sin(q_2) \\ 0 \end{bmatrix} \implies \dot{r}_{c2}^0 = \begin{bmatrix} \dot{q}_1 - l_c \sin(q_2)\dot{q}_2 \\ l_c \cos(q_2)\dot{q}_2 \\ 0 \end{bmatrix}$$

The angular velocities are:
*   Cart: $\omega_1^0 = 0$
*   Rod: $\omega_2^0 = \begin{bmatrix} 0 & 0 & \dot{q}_2 \end{bmatrix}^T$

### 2. Kinetic Energy ($K$) and Mass Matrix ($D(q)$)
Substituting the velocities into the kinetic energy formula:
$$K = \frac{1}{2} M_1 \dot{q}_1^2 + \frac{1}{2} M_2 \left[ (\dot{q}_1 - l_c \sin(q_2)\dot{q}_2)^2 + (l_c \cos(q_2)\dot{q}_2)^2 \right] + \frac{1}{2} I_z \dot{q}_2^2$$
Expanding and simplifying the squared terms gives:
$$K = \frac{1}{2} M_1 \dot{q}_1^2 + \frac{1}{2} M_2 \left[ \dot{q}_1^2 - 2 l_c \sin(q_2)\dot{q}_1 \dot{q}_2 + (l_c \dot{q}_2)^2 \sin^2(q_2) + (l_c \dot{q}_2)^2 \cos^2(q_2) \right] + \frac{1}{2} \dot{q}_2^2 I_z$$
Because $\sin^2(q_2) + \cos^2(q_2) = 1$, we can rewrite $K$ in the standard quadratic form $K(q, \dot{q}) = \frac{1}{2} \dot{q}^T D(q) \dot{q}$:
$$K = \frac{1}{2} \begin{bmatrix} \dot{q}_1 & \dot{q}_2 \end{bmatrix} \underbrace{\begin{bmatrix} M_1 + M_2 & -M_2 l_c \sin(q_2) \\ -M_2 l_c \sin(q_2) & M_2 l_c^2 + I_z \end{bmatrix}}_{D(q): \text{ Mass Matrix}} \begin{bmatrix} \dot{q}_1 \\ \dot{q}_2 \end{bmatrix}$$
*(Note: $I_z$ denotes $(I_2)_{33}$, the moment of inertia of the rod about the z-axis).*

### 3. Potential Energy ($P$)
Based on the vertical height (y-coordinates) of the centers of mass:
$$P(q) = M_1 g a_1 + M_2 g (a_1 + a_2 + l_c \sin(q_2))$$

### 4. Euler-Lagrange Equations
The Lagrangian is $\mathcal{L}(q, \dot{q}) = K(q, \dot{q}) - P(q)$. 

*(Error Correction: The handwritten notes contain a typo, writing the Euler-Lagrange equation as $\frac{d}{dt}\nabla_{\dot{q}}\mathcal{L} - \nabla_q \mathcal{L} - P(q)$. Because $P(q)$ is already included in $\mathcal{L}$, the standard and correct form is simply $\frac{d}{dt}(\nabla_{\dot{q}}\mathcal{L}) - \nabla_q \mathcal{L} = \tau$. The mathematical execution in the notes correctly follows this standard form).*

**Term 1: $\frac{d}{dt} \nabla_{\dot{q}} \mathcal{L}$**
Since $\nabla_{\dot{q}} \mathcal{L} = D(q)\dot{q}$, we use the product rule:
$$\frac{d}{dt}(D(q)\dot{q}) = \dot{D}(q)\dot{q} + D(q)\ddot{q}$$
Taking the time derivative of the mass matrix $D(q)$:
$$\dot{D}(q)\dot{q} = \begin{bmatrix} 0 & -M_2 l_c \cos(q_2)\dot{q}_2 \\ -M_2 l_c \cos(q_2)\dot{q}_2 & 0 \end{bmatrix} \begin{bmatrix} \dot{q}_1 \\ \dot{q}_2 \end{bmatrix} = \begin{bmatrix} -M_2 l_c \cos(q_2)\dot{q}_2^2 \\ -M_2 l_c \cos(q_2)\dot{q}_1 \dot{q}_2 \end{bmatrix}$$

**Term 2: $\nabla_q \mathcal{L}$**
Taking the gradient of the Lagrangian w.r.t $q_1$ and $q_2$:
$$\nabla_q \mathcal{L} = \nabla_q \left[ \frac{1}{2} \dot{q}_1^2 (M_1 + M_2) + \frac{1}{2} \dot{q}_2^2 (l_c^2 M_2 + I_z) - M_2 \dot{q}_1 \dot{q}_2 l_c \sin(q_2) - M_2 g l_c \sin(q_2) \right]$$
$$\nabla_q \mathcal{L} = \begin{bmatrix} 0 \\ -M_2 \dot{q}_1 \dot{q}_2 l_c \cos(q_2) - M_2 g l_c \cos(q_2) \end{bmatrix}$$

**Final Equations of Motion ($\Box - \Box = \tau$)**
*To fill in the gaps left by the blank boxes in the notes, we combine the terms:*
$$\left( \dot{D}(q)\dot{q} + D(q)\ddot{q} \right) - \nabla_q \mathcal{L} = \begin{bmatrix} \tau_1 \\ \tau_2 \end{bmatrix}$$
Substituting the matrices and simplifying yields the two equations of motion:
1.  **Cart:** $(M_1 + M_2)\ddot{q}_1 - M_2 l_c \sin(q_2)\ddot{q}_2 - M_2 l_c \cos(q_2)\dot{q}_2^2 = \tau_1$
2.  **Rod:** $-M_2 l_c \sin(q_2)\ddot{q}_1 + (M_2 l_c^2 + I_z)\ddot{q}_2 + M_2 g l_c \cos(q_2) = \tau_2$ *(Note how the Coriolis terms involving $\dot{q}_1\dot{q}_2$ canceled out).*

---

## Approach 2: Christoffel Symbols

Instead of taking the gradient of the Lagrangian, we can systematically find the Coriolis and Centrifugal matrix $C(q, \dot{q})$ using Christoffel symbols.

First, identify all partial derivatives of the mass matrix elements $\frac{\partial D_{ij}}{\partial q_k}$:
*   $\frac{\partial D_{11}}{\partial q_1} = \frac{\partial D_{11}}{\partial q_2} = 0$
*   $\frac{\partial D_{22}}{\partial q_1} = \frac{\partial D_{22}}{\partial q_2} = 0$
*   $\frac{\partial D_{12}}{\partial q_1} = \frac{\partial D_{21}}{\partial q_1} = 0$
*   **$\frac{\partial D_{12}}{\partial q_2} = \frac{\partial D_{21}}{\partial q_2} = -M_2 l_c \cos(q_2)$** *(This is the only non-zero derivative)*

The notes list `ijk = 221, 212, 122`. These correspond to the specific Christoffel symbols $c_{ijk}$ that involve the non-zero derivative. Let's compute the elements of the $C$ matrix to complete the notes.
The formula for a Christoffel symbol is: $c_{ijk} = \frac{1}{2} \left( \frac{\partial D_{ij}}{\partial q_k} + \frac{\partial D_{ik}}{\partial q_j} - \frac{\partial D_{jk}}{\partial q_i} \right)$
*(where $i$ is the row of $C$, $j$ is the column of $C$, and $k$ is the velocity index $\dot{q}_k$)*

Calculating the relevant symbols:
*   $c_{122} = \frac{1}{2} \left( \frac{\partial D_{12}}{\partial q_2} + \frac{\partial D_{12}}{\partial q_2} - \frac{\partial D_{22}}{\partial q_1} \right) = \frac{1}{2} (-M_2 l_c \cos q_2 - M_2 l_c \cos q_2 - 0) = -M_2 l_c \cos(q_2)$
*   $c_{212} = \frac{1}{2} \left( \frac{\partial D_{21}}{\partial q_2} + \frac{\partial D_{22}}{\partial q_1} - \frac{\partial D_{12}}{\partial q_2} \right) = \frac{1}{2} (-M_2 l_c \cos q_2 + 0 - (-M_2 l_c \cos q_2)) = 0$
*   $c_{221} = \frac{1}{2} \left( \frac{\partial D_{22}}{\partial q_1} + \frac{\partial D_{21}}{\partial q_2} - \frac{\partial D_{12}}{\partial q_2} \right) = 0$
*(All other $c_{ijk}$ evaluate to 0)*

Constructing the $C(q, \dot{q})$ matrix using $C_{ij} = \sum_{k} c_{ijk} \dot{q}_k$:
$$C(q, \dot{q}) = \begin{bmatrix} 0 & -M_2 l_c \cos(q_2)\dot{q}_2 \\ 0 & 0 \end{bmatrix}$$

Multiplying this by $\dot{q}$ yields the Coriolis/Centrifugal vector:
$$C(q, \dot{q})\dot{q} = \begin{bmatrix} -M_2 l_c \cos(q_2)\dot{q}_2^2 \\ 0 \end{bmatrix}$$
This perfectly matches the $\dot{D}\dot{q} - \nabla_q K$ terms derived in Approach 1, proving both methods yield the exact same dynamic equations.