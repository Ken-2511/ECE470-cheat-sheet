Here is a comprehensive summary of your lecture notes. I have structured the content, transcribed the equations into LaTeX, and added a few important corrections and missing intermediate steps to ensure the math is logically complete and accurate.

---

# ECE470 Robotics: Lecture Notes Summary

## 1. Slotine-Li Non-Adaptive Controller (Continued)
**Recall from last time:**
We define a combined error metric (filtering term) $r$:
$$r = \dot{\tilde{q}} + \Lambda \tilde{q}$$
The closed-loop dynamics of the system with the Slotine-Li controller is given by equation (3):
$$M\dot{r} + Cr + Kr = 0 \quad \text{--- (3)}$$

### Theorem
The equilibrium point $(\tilde{q}, \dot{\tilde{q}}) = (0,0)$ is **Asymptotically Stable (A.S.)**.
*(Proof is due to Ortega, Spong, and Kelly).*

### Proof of Asymptotic Stability
We define a Lyapunov function candidate:
$$V(\tilde{q}, \dot{\tilde{q}}) = \frac{1}{2} r^T M(q) r + \frac{1}{2} \tilde{q}^T P \tilde{q}$$
*(Note: $P = P^T$ is a symmetric matrix to be determined).*

The proof proceeds in 3 steps:

**Step 1: Check that $(\tilde{q}, \dot{\tilde{q}}) = (0,0)$ is an equilibrium of (3).**
Let $x_1 = \tilde{q}$ and $x_2 = \dot{\tilde{q}}$. We can write the state equations:
$$ \begin{cases} \dot{x}_1 = x_2 \\ \dot{x}_2 = \ddot{\tilde{q}} = f(t, \tilde{q}, \dot{\tilde{q}}) \end{cases} $$
If we plug in $(\tilde{q}, \dot{\tilde{q}}) = (0,0)$, then $r = 0$. Substituting $r=0$ into (3) yields $M\dot{r} = 0 \implies \dot{r}=0$.
Since $\dot{r} = \ddot{\tilde{q}} + \Lambda\dot{\tilde{q}}$, and $\dot{\tilde{q}}=0$, we get $\ddot{\tilde{q}} = 0$. Thus, $f(t, 0, 0) = 0$, confirming it is an equilibrium.

**Step 2: Show $V$ is positive definite (p.d.) at $(\tilde{q}, \dot{\tilde{q}}) = (0,0)$.**
$V$ is the sum of two terms. For $V \ge 0$, both terms must be $\ge 0$.
$$V = 0 \iff r^T M r = 0 \quad \text{and} \quad \tilde{q}^T P \tilde{q} = 0 \iff \tilde{q} = 0, \dot{\tilde{q}} = 0$$
*(Requires choosing $P$ to be positive definite).*

**Step 3: Show $\dot{V}$ is negative definite (n.d.).**
Take the time derivative of $V$:
$$\dot{V} = \frac{d}{dt}\left(\frac{1}{2} r^T M r\right) + \frac{d}{dt}\left(\frac{1}{2} \tilde{q}^T P \tilde{q}\right)$$

> **TA Note (Missing Gap Fill):** The notes skip how the first term simplifies. Using the product rule:
> $\frac{d}{dt}\left(\frac{1}{2} r^T M r\right) = r^T M \dot{r} + \frac{1}{2} r^T \dot{M} r$
> Substitute $M\dot{r} = -Cr - Kr$ from Eq (3):
> $= r^T(-Cr - Kr) + \frac{1}{2}r^T\dot{M}r = -r^T K r + \frac{1}{2} r^T (\dot{M} - 2C) r$
> By the fundamental skew-symmetric property of robots, $x^T(\dot{M}-2C)x = 0$. Therefore, the entire first term reduces simply to $-r^T K r$.

Continuing with the derivative:
$$\dot{V} = -r^T K r + \tilde{q}^T P \dot{\tilde{q}}$$
Substitute $r = \dot{\tilde{q}} + \Lambda \tilde{q}$:
$$\dot{V} = -(\dot{\tilde{q}} + \Lambda \tilde{q})^T K (\dot{\tilde{q}} + \Lambda \tilde{q}) + \tilde{q}^T P \dot{\tilde{q}}$$
Expanding the first term:
$$\dot{V} = -\dot{\tilde{q}}^T K \dot{\tilde{q}} - \tilde{q}^T \Lambda K \Lambda \tilde{q} \underbrace{- 2\tilde{q}^T \Lambda K \dot{\tilde{q}} + \tilde{q}^T P \dot{\tilde{q}}}_{\text{Cross terms}}$$
To eliminate the cross terms, we **choose $P = 2\Lambda K$**. This leaves:
$$\dot{V} = -\dot{\tilde{q}}^T K \dot{\tilde{q}} - \tilde{q}^T \Lambda K \Lambda \tilde{q}$$

We can write this as a quadratic form in matrix notation:
$$\dot{V} = - \begin{bmatrix} \tilde{q} \\ \dot{\tilde{q}} \end{bmatrix}^T \begin{bmatrix} \Lambda K \Lambda & 0 \\ 0 & K \end{bmatrix} \begin{bmatrix} \tilde{q} \\ \dot{\tilde{q}} \end{bmatrix}$$
Because the central block matrix is positive definite (p.d.), the negative sign makes $\dot{V}$ **negative definite (n.d.)** at $(\tilde{q}, \dot{\tilde{q}}) \neq (0,0)$.
This completes the proof that the equilibrium is Asymptotically Stable.

---

## 2. Slotine-Li Adaptive Controller
Assume that the inertia parameters of the robot are unknown (e.g., Masses, Moment of inertia, gravity $g$, spring constants, friction coefficients, displacements of the Center of Mass).

**Key Observation:**
For the standard robot dynamics equation $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$, it is possible to group the unknown physical parameters into a vector $\Theta \in \mathbb{R}^l$ of "parameter clusters", so that the dynamics can be rewritten linearly with respect to these unknowns as:
$$Y(q, \dot{q}, \ddot{q}) \Theta = \tau$$
Where $Y(q, \dot{q}, \ddot{q})$ is a matrix of known kinematics called the **Regressor Matrix**.

### Example 1: Cart-Pendulum
The dynamic equations are given in matrix form:
$$\begin{bmatrix} M & -M_2 l_{c2} c_{q_2} \\ -M_2 l_{c2} c_{q_2} & I \end{bmatrix} \begin{bmatrix} \ddot{q}_1 \\ \ddot{q}_2 \end{bmatrix} + \begin{bmatrix} 0 & M_2 l_{c2} s_{q_2} \dot{q}_2 \\ 0 & 0 \end{bmatrix} \begin{bmatrix} \dot{q}_1 \\ \dot{q}_2 \end{bmatrix} + \begin{bmatrix} 0 \\ -M_2 l_{c2} g s_{q_2} \end{bmatrix} + \begin{bmatrix} b_{11} & b_{12} \\ b_{12} & b_{22} \end{bmatrix} \begin{bmatrix} \dot{q}_1 \\ \dot{q}_2 \end{bmatrix} = \tau$$
> **TA Correction:** In your notes, the bottom-left entry of the mass matrix looks like it was written as $-M_2 l_{c2} s_{q_2}$. The mass matrix **must be symmetric**, so it should correctly be $-M_2 l_{c2} \cos(q_2)$ or $-M_2 l_{c2} c_{q_2}$ as fixed above.

We define the vector of unknown parameter clusters $\Theta \in \mathbb{R}^6$:
$\Theta = [\theta_1, \theta_2, \theta_3, \theta_4, \theta_5, \theta_6]^T = [M, M_2 l_{c2}, I, b_{11}, b_{12}, b_{22}]^T$

Expanding the equations row-by-row gives:
1. $\theta_1 \ddot{q}_1 - \theta_2 c_{q_2} \ddot{q}_2 + \theta_2 s_{q_2} \dot{q}_2^2 + \theta_4 \dot{q}_1 + \theta_5 \dot{q}_2 = \tau_1$
2. $-\theta_2 c_{q_2} \ddot{q}_1 + \theta_3 \ddot{q}_2 - \theta_2 g s_{q_2} + \theta_5 \dot{q}_1 + \theta_6 \dot{q}_2 = \tau_2$ *(Filled in missing line from notes)*

We can now construct the Regressor equation $Y\Theta = \tau$:
$$\underbrace{\begin{bmatrix} \ddot{q}_1 & -c_{q_2}\ddot{q}_2 + s_{q_2}\dot{q}_2^2 & 0 & \dot{q}_1 & \dot{q}_2 & 0 \\ 0 & -c_{q_2}\ddot{q}_1 - g s_{q_2} & \ddot{q}_2 & 0 & \dot{q}_1 & \dot{q}_2 \end{bmatrix}}_{Y(q,\dot{q},\ddot{q})} \underbrace{\begin{bmatrix} \theta_1 \\ \theta_2 \\ \theta_3 \\ \theta_4 \\ \theta_5 \\ \theta_6 \end{bmatrix}}_{\Theta} = \begin{bmatrix} \tau_1 \\ \tau_2 \end{bmatrix}$$
> **TA Correction:** In the second row of the $Y$ matrix in your notes, the term was written as $-\ddot{q}_1 - g s_{q_2}$. It is missing the $c_{q_2}$ term. It should be $-c_{q_2}\ddot{q}_1 - g s_{q_2}$, which I have corrected above.

---

### Example 2: RR Robot (No Friction)
Dynamics: $D(q)\ddot{q} + C(q,\dot{q})\dot{q} + \nabla_q P = u$

**Inertia Matrix $D(q)$ components:**
> **TA Correction:** Your notes have a dimensional error in $d_{11}(q)$, writing $M_2(l_1 + l_{c2}^2)$. Lengths and squared lengths cannot be added. Referencing the blackboard photo, it should be $l_1^2 + l_{c2}^2$. The standard parameter groupings for an RR robot are:
> $\theta_1 = M_1 l_{c1}^2 + M_2 l_1^2 + I_1$
> $\theta_2 = M_2 l_{c2}^2 + I_2$
> $\theta_3 = M_2 l_1 l_{c2}$

Using these standard parameters, the matrix elements become:
*   $d_{11}(q) = \theta_1 + \theta_2 + 2\theta_3 \cos(q_2)$
*   $d_{12}(q) = d_{21}(q) = \theta_2 + \theta_3 \cos(q_2)$
*   $d_{22}(q) = \theta_2$

**Coriolis Matrix $C(q,\dot{q})$:**
$$C = -M_2 l_1 l_{c2} s_{q_2} \begin{bmatrix} \dot{q}_2 & \dot{q}_1 + \dot{q}_2 \\ -\dot{q}_1 & 0 \end{bmatrix}$$

**Extracting the Regressor Matrix $Y$ (Exercise Check):**
The notes show the process of isolating the $\theta_3$ term for the first row to build the Regressor matrix.
Looking at the torque equation for Joint 1 ($\tau_1$):
$\tau_1 = d_{11}\ddot{q}_1 + d_{12}\ddot{q}_2 + C_{11}\dot{q}_1 + C_{12}\dot{q}_2 + g_1$

If we gather all terms containing $\theta_3$ (which represents $M_2 l_1 l_{c2}$):
*   From $d_{11}\ddot{q}_1$: $2 c_{q_2} \ddot{q}_1$
*   From $d_{12}\ddot{q}_2$: $c_{q_2} \ddot{q}_2$
*   From Coriolis: $-s_{q_2} \dot{q}_2 \dot{q}_1 - s_{q_2} (\dot{q}_1 + \dot{q}_2)\dot{q}_2$

Factoring out $c_{q_2}$ and $s_{q_2}$ confirms the column entry in the $Y$ matrix for $\theta_3$ as shown at the bottom of your notes:
$$Y_{1,3} = c_{q_2}(2\ddot{q}_1 + \ddot{q}_2) - s_{q_2}\dot{q}_1\dot{q}_2 - s_{q_2}(\dot{q}_1 + \dot{q}_2)\dot{q}_2$$