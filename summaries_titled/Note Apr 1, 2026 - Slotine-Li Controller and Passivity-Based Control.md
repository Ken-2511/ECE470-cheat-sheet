Here is a comprehensive and structured summary of your lecture notes. I have organized the content, formatted all mathematics using LaTeX, and added a few "TA Notes" to clarify missing steps, fix minor notation issues, and make the derivations completely self-contained.

---

# Lecture Notes Summary: Slotine-Li Controller (Passivity-Based Control)

## 1. Recap: PD Controller with Gravity Compensation
The lecture begins by recalling the PD controller from the previous session.
The standard robot dynamics (plant) is given by:
$$ M(q)\ddot{q} + C(q,\dot{q})\dot{q} + \nabla_q P + K_f(q)\dot{q} = u \quad \text{(1)} $$
*Where:*
*   $M(q)$: Inertia matrix
*   $C(q,\dot{q})$: Coriolis/centrifugal matrix
*   $\nabla_q P$: Gravity vector (gradient of potential energy $P$)
*   $K_f(q)\dot{q}$: Friction term *(Note: The notes write this as $K(q)\dot{q}$ initially, but later clarify it as $K_{\text{friction}}\dot{q}$)*
*   $u$: Control input torques

**Error Definition:**
We define the position error as $\tilde{q} := q^r - q$, where $q^r$ is the reference trajectory.
For a PD controller, we pick gain matrices that are symmetric and positive definite (p.d.):
*   $K_p = K_p^T > 0$
*   $K_d = K_d^T > 0$

---

## 2. Temporary Controller (Assuming Perfect Knowledge & No Friction)
To build up to the **Slotine-Li Controller (1988)** / **Ortega-Spong (1989)**, we first try a temporary control law. We assume we perfectly know the robot dynamics and that there is **no friction**.

**Proposed Control Law:**
$$ u = M(q)\ddot{q}^r + C(q,\dot{q})\dot{q}^r + \nabla_q P + K\dot{\tilde{q}} \quad \text{(2)} $$

**Closed-Loop Dynamics:**
Substitute the controller (2) into the plant (1) (assuming friction $K_f = 0$):
$$ M(q)\ddot{q} + C(q,\dot{q})\dot{q} + \nabla_q P = M(q)\ddot{q}^r + C(q,\dot{q})\dot{q}^r + \nabla_q P + K\dot{\tilde{q}} $$

Cancel gravity $\nabla_q P$ on both sides and group the terms:
$$ M(q)(\underbrace{\ddot{q}^r - \ddot{q}}_{\ddot{\tilde{q}}}) + C(q,\dot{q})(\underbrace{\dot{q}^r - \dot{q}}_{\dot{\tilde{q}}}) + K\dot{\tilde{q}} = 0 \quad \text{(3)} $$

Let's define a new variable **$r = \dot{\tilde{q}}$**. This simplifies equation (3) into our target Ordinary Differential Equation (ODE):
$$ M(q)\dot{r} + C(q,\dot{q})r + Kr = 0 \quad \text{(4)} $$

---

## 3. Stability Analysis of the Temporary Controller
We want to prove that the error velocity converges to zero: $r(t) \to 0$.
Define the **Lyapunov function**:
$$ V = \frac{1}{2} r^T M(q) r $$

Take the time derivative $\dot{V}$:
$$ \dot{V} = \frac{1}{2} r^T \dot{M} r + r^T M(q)\dot{r} $$

From equation (4), isolate $M(q)\dot{r}$:
$$ M(q)\dot{r} = -C(q,\dot{q})r - Kr $$

Substitute this back into $\dot{V}$:
$$ \dot{V} = \frac{1}{2} r^T \dot{M} r + r^T (-Cr - Kr) $$
$$ \dot{V} = \frac{1}{2} r^T (\dot{M} - 2C) r - r^T K r $$

> **TA Note (Crucial Property):** A fundamental property of robot dynamics is that the matrix $(\dot{M} - 2C)$ is **skew-symmetric**. Therefore, for any vector $x$, $x^T(\dot{M} - 2C)x = 0$.

Applying the skew-symmetry property:
$$ \frac{1}{2} r^T (\dot{M} - 2C) r = 0 $$
$$ \dot{V} = -r^T K r $$

Because $K$ is positive definite, $\dot{V} \le 0$. By the **Krasovskii-LaSalle (KL) Invariance Principle**, as $t \to \infty$, $\dot{V} \to 0 \iff r \to 0$.

### The Flaw with the Temporary Controller
We proved that $r = \dot{\tilde{q}}(t) \to 0$. **However, $\dot{\tilde{q}}(t) \to 0$ does NOT guarantee that the position error $\tilde{q}(t) \to 0$.** The robot could stop at a constant position offset.

---

## 4. The Fix: Filtered Tracking Error
To fix the issue, we redefine $r$ from a simple velocity error to a **"filtered tracking error"**:
$$ r := \dot{\tilde{q}} + \Lambda \tilde{q} $$
Where $\Lambda$ is a diagonal matrix of positive gains: $\Lambda = \text{diag}(\lambda_1, \dots, \lambda_n)$ with $\lambda_i > 0$.

### Why does this work?
If we can force $r(t) \to 0$, then we get:
$$ \dot{\tilde{q}} + \Lambda \tilde{q} \to 0 \implies \dot{\tilde{q}}_i \to -\lambda_i \tilde{q}_i $$
In steady state, this is a simple first-order ODE: $\dot{\tilde{q}}_i = -\lambda_i \tilde{q}_i$.
The solution is **$\tilde{q}_i(t) = e^{-\lambda_i t} \tilde{q}_i(0)$**, which guarantees that the position error $\tilde{q}(t)$ exponentially decays to 0.

> **Phase Portrait Interpretation:** In the error state space $(\tilde{q}, \dot{\tilde{q}})$, the equation $r=0$ (or $\dot{\tilde{q}} = -\Lambda \tilde{q}$) represents a straight line through the origin. The controller drives the system onto this line (a "sliding surface"), and the system slides down the line to the origin $(0,0)$.

---

## 5. The Full Slotine-Li Controller (Non-Adaptive)
Now we construct the actual control law $u$ that induces the stable ODE (Equation 4) using the new definition of $r$, while also accounting for friction.

**Plant (with friction):**
$$ M\ddot{q} + C\dot{q} + K_{\text{fric}}\dot{q} + \nabla_q P = u $$

**Target ODE Form:**
$$ M\dot{r} + Cr + Kr = 0 $$

> **TA Note (Derivation Gap Filled):** The notes have a "?" when finding $\dot{r}$. Here is the exact derivation.
> Since $r = \dot{\tilde{q}} + \Lambda \tilde{q}$, the derivative is:
> $$ \dot{r} = \ddot{\tilde{q}} + \Lambda \dot{\tilde{q}} = (\ddot{q}^r - \ddot{q}) + \Lambda \dot{\tilde{q}} $$
> Substitute $r$ and $\dot{r}$ into the target form $M\dot{r} + Cr + Kr = 0$:
> $$ M(\ddot{q}^r - \ddot{q} + \Lambda \dot{\tilde{q}}) + C(\dot{q}^r - \dot{q} + \Lambda \tilde{q}) + Kr = 0 $$
> Expand and rearrange to isolate $M\ddot{q} + C\dot{q}$:
> $$ M\ddot{q} + C\dot{q} = M(\ddot{q}^r + \Lambda \dot{\tilde{q}}) + C(\dot{q}^r + \Lambda \tilde{q}) + Kr $$
> Finally, substitute this result into the plant $u = (M\ddot{q} + C\dot{q}) + K_{\text{fric}}\dot{q} + \nabla_q P$.

**Final Control Law:**
$$ u = K_{\text{fric}}(q)\dot{q} + \nabla_q P + M(q)(\ddot{q}^r + \Lambda \dot{\tilde{q}}) + C(q,\dot{q})(\dot{q}^r + \Lambda \tilde{q}) + K(\dot{\tilde{q}} + \Lambda \tilde{q}) $$

*(Note: The last term $K(\dot{\tilde{q}} + \Lambda \tilde{q})$ is simply $Kr$)*. 
This controller guarantees global asymptotic stability for the tracking error $\tilde{q}(t) \to 0$.