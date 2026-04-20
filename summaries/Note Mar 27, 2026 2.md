Here is a comprehensive summary of the ECE470 lecture notes on robot control, structured with clear headings, bullet points, and LaTeX equations. 

I have transcribed the content, filled in missing mathematical gaps (such as completing the proofs and examples shown partially on the boards), and corrected minor notation issues for mathematical completeness.

---

# ECE470 Tutorial 11: Robot Control

## 1. System Dynamics & Control Specification
The standard dynamics of a robotic manipulator are given by:
$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + \nabla_q P(q) + B(q)\dot{q} = u$$
Where:
*   $M(q)$ is the mass/inertia matrix.
*   $C(q,\dot{q})\dot{q}$ contains Coriolis and centrifugal terms.
*   $\nabla_q P(q)$ is the gravity vector.
*   $B(q)\dot{q}$ represents friction/damping.
*   $u$ is the control input (torque).

**Control Specification:** 
Design a controller to make the actual trajectory $q(t)$ track a reference trajectory $q^r(t)$, where $q^r(t)$ is a twice-differentiable reference signal.
*   **Position Error:** $\tilde{q} = q^r(t) - q$
*   **Velocity Error:** $\dot{\tilde{q}} = \dot{q}^r(t) - \dot{q}$

---

## 2. Control Methods

### Method 1: Computed Torque Method (Feedback Linearization)
This method perfectly cancels the nonlinear dynamics of the system and imposes desired linear error dynamics.
$$u = M(q)a + C(q,\dot{q})\dot{q} + \nabla_q P(q) + B(q)\dot{q}$$
Substituting this into the system dynamics yields $\ddot{q} = a$. 

To achieve tracking, we design $a$ as a Proportional-Derivative (PD) controller with feed-forward acceleration:
$$a = \ddot{q}^r(t) + K_p \tilde{q} + K_d \dot{\tilde{q}}$$
Where $K_p$ and $K_d$ are positive definite diagonal gain matrices:
$K_p = \begin{bmatrix} k_p^1 & 0 \\ 0 & \ddots \end{bmatrix} > 0, \quad K_d = \begin{bmatrix} k_d^1 & 0 \\ 0 & \ddots \end{bmatrix} > 0$

### Method 2: PD Controller with Gravity Compensation
Used strictly when the reference trajectory $q^r$ is **constant** (setpoint regulation, meaning $\dot{q}^r = 0$ and $\ddot{q}^r = 0$).
$$u = \nabla_q P(q) + K_p \tilde{q} + K_d \dot{\tilde{q}}$$
*   Asymptotic stability of the equilibrium $(q, \dot{q}) = (q^r, 0)$ is proved using the Lyapunov function $V(q, \dot{q}) = \frac{1}{2}\dot{q}^T M(q) \dot{q} + \frac{1}{2}\tilde{q}^T K_p \tilde{q}$ and the Krasovskii-LaSalle (K-L) Invariance Principle.

### Method 3: Slotine-Li "Passivity-Based" Controller
A more robust tracking controller that does not require full inversion of the mass matrix for stability.
$$u = M(q)(\ddot{q}^r + \Lambda \dot{\tilde{q}}) + C(q, \dot{q})(\dot{q}^r + \Lambda \tilde{q}) + B(q)\dot{q} + \nabla_q P(q) + K(\dot{\tilde{q}} + \Lambda \tilde{q})$$
Where:
*   $K = K^T$ is positive definite (p.d.).
*   $\Lambda = \begin{bmatrix} \lambda_1 & 0 \\ 0 & \lambda_n \end{bmatrix}$, with $\lambda_i > 0$.
*   We define a composite error metric (sliding surface): $r = \dot{\tilde{q}} + \Lambda \tilde{q}$
*   Lyapunov function: $V = \frac{1}{2} r^T M(q) r$

By proving $\dot{V} \leq 0$, we can show $r(t) \to 0$ as $t \to \infty$. 
When $r(t) \equiv 0$, then:
$$\dot{\tilde{q}} + \Lambda \tilde{q} = 0 \implies \dot{\tilde{q}}_i = -\lambda_i \tilde{q}_i$$
*Correction to notes:* The notes state the solution is $\tilde{q}_i(t) = e^{-\lambda_i t}$. To be mathematically precise, it requires the initial condition: **$\tilde{q}_i(t) = \tilde{q}_i(0) e^{-\lambda_i t}$**. Because $\lambda_i > 0$, $\tilde{q}_i(t) \to 0$ exponentially.

---

## 3. Worked Examples

### Example A: 2017 Final Q4 (Computed Torque)
**System:** $m\ddot{x} + b\dot{x} = u \quad (x \in \mathbb{R})$
**Goal:** Let $x^r(t) = 10 + \sin(t)$. Design a "Method 1" controller making $x^r(t) - x(t) \to 0$ as $t \to \infty$.

*Note: The notes prompt the question. Here is the completed solution based on Method 1:*
1.  Identify terms: $M(x) = m$, $B(x) = b$, $C=0$, $P=0$.
2.  Control law: $u = m \cdot a + b\dot{x}$
3.  Design $a$: $a = \ddot{x}^r(t) + K_p(x^r - x) + K_d(\dot{x}^r - \dot{x})$
4.  Derive references: $\dot{x}^r(t) = \cos(t)$ and $\ddot{x}^r(t) = -\sin(t)$.
5.  **Final Controller:** $u = m(-\sin(t) + K_p(10 + \sin(t) - x) + K_d(\cos(t) - \dot{x})) + b\dot{x}$ (with $K_p, K_d > 0$).

### Example B: 2024 Final Q2 (PD + Gravity Comp & Stability)
**System (Pendulum):** $ml^2\ddot{\theta} + mgl\sin\theta = \tau$
**Goal:** Design a "Method 2" controller to make $(\theta, \dot{\theta}) \to (\pi, 0)$. 

#### Part (a): Design the Controller
1.  Identify components: $M(q) = ml^2$, $\nabla_q P = mgl\sin\theta$, $u = \tau$.
2.  Errors: $\theta^r = \pi$. Therefore, $\tilde{\theta} = \pi - \theta$ and $\dot{\tilde{\theta}} = 0 - \dot{\theta} = -\dot{\theta}$.
3.  Apply Method 2: $u = \nabla_q P + K_p \tilde{q} + K_d \dot{\tilde{q}}$
    $$u = mgl\sin\theta + K_p(\pi - \theta) + K_d(-\dot{\theta})$$
4.  Choose numerical gains: e.g., $K_p = 2, K_d = 5$.

#### Part (b): Stability Analysis using Krasovskii-LaSalle
**Consider the Closed Loop System (CLS):**
$$ml^2\ddot{\theta} + mgl\sin\theta = mgl\sin\theta + K_p(\pi - \theta) - K_d\dot{\theta}$$
Canceling gravity terms gives the CLS dynamics:
$$ml^2\ddot{\theta} = K_p(\pi - \theta) - K_d\dot{\theta}$$

**Krasovskii-LaSalle (K-L) Invariance Theorem (Definition from notes):**
Let $\bar{x}$ be an equilibrium point of $\dot{x} = f(x)$ (i.e., $f(\bar{x}) = 0$). Suppose there exists a differentiable function $V: \mathbb{R}^n \to \mathbb{R}$ which is positive definite (p.d.) at $\bar{x}$ and such that $\dot{V} = \frac{\partial V}{\partial x}f(x)$ is negative semi-definite (n.s.d.). Then $\bar{x}$ is stable and $\dot{V}(x(t)) \to 0$ as $t \to \infty$. Moreover, if $\dot{V}(x(t)) \equiv 0$ implies $x(t) \equiv \bar{x}$, then $\bar{x}$ is asymptotically stable.

**4-Step Proof for Asymptotic Stability of $(\pi, 0)$:**

**Step 1: Find V (Lyapunov Function)**
$$V(\theta, \dot{\theta}) = \frac{1}{2}\dot{\theta}(M(q))\dot{\theta} + \frac{1}{2}(\pi - \theta)^2 K_p = \frac{1}{2}ml^2\dot{\theta}^2 + \frac{1}{2}K_p(\pi - \theta)^2$$

**Step 2: Check V is Positive Definite @ $\bar{x} = (\pi, 0)$**
*   $V(\pi, 0) = \frac{1}{2}ml^2(0)^2 + \frac{1}{2}K_p(\pi - \pi)^2 = 0 \quad \checkmark$
*   For any point other than $(\pi, 0)$, $V > 0$ (since it's a sum of squares multiplied by positive constants).

**Step 3: Check $\dot{V}$ is Negative Semi-Definite @ $\bar{x}$**
Differentiate $V$ with respect to time:
$$\dot{V} = ml^2\dot{\theta}\ddot{\theta} + K_p(\pi - \theta)(-\dot{\theta})$$
Substitute the CLS dynamics ($ml^2\ddot{\theta}$) into the equation:
$$\dot{V} = [K_p(\pi - \theta) - K_d\dot{\theta}]\dot{\theta} - K_p(\pi - \theta)\dot{\theta}$$
$$\dot{V} = K_p(\pi - \theta)\dot{\theta} - K_d\dot{\theta}^2 - K_p(\pi - \theta)\dot{\theta}$$
$$\dot{V} = -K_d\dot{\theta}^2 \leq 0 \quad \checkmark \text{ (n.s.d.)}$$

**Step 4: Set $\dot{V} \equiv 0$ and check if it implies $x \equiv \bar{x}$**
If $\dot{V} \equiv 0$, then:
$$-K_d\dot{\theta}^2 = 0 \implies \dot{\theta} \equiv 0 \implies \ddot{\theta} \equiv 0$$
Substitute $\dot{\theta} = 0$ and $\ddot{\theta} = 0$ back into the CLS dynamics:
$$ml^2(0) = K_p(\pi - \theta) - K_d(0)$$
$$0 = K_p(\pi - \theta)$$
Since $K_p > 0$, this implies $\theta \equiv \pi$.
Because $\dot{V} \equiv 0$ strictly implies the system is at the equilibrium point $(\theta, \dot{\theta}) = (\pi, 0)$, by the Krasovskii-LaSalle Invariance Principle, the equilibrium point is **asymptotically stable**.