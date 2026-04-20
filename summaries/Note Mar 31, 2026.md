Here is a structured Markdown summary of the provided lecture notes. 

***

# ECE470 Robotics: Stability Theory and PD Control

## 1. Recap: Krasovsky-LaSalle (KL) Theorem
The notes begin with a review of the rudiments of stability theory, specifically the Krasovsky-LaSalle (KL) Theorem.

**System Definition:** Let $\dot{x} = f(x)$, where $\bar{x}$ is an equilibrium point, meaning $f(\bar{x}) = 0$.

**Theorem Statement:** Suppose there exists a continuously differentiable ($C^1$) function $V: \mathbb{R}^n \to \mathbb{R}$ that is positive definite (p.d.) at $\bar{x}$, such that:
$$ \dot{V} = \frac{\partial V}{\partial x} f(x) \le 0 $$
*(Meaning $V$ is non-increasing, and typically $\dot{V} \to 0$ as $t \to \infty$)*. 

Moreover, **if** $\dot{V}(x(t)) \equiv 0 \implies x(t) \equiv \bar{x}$, **then** the equilibrium point $\bar{x}$ is **Asymptotically Stable (AS)**.

---

## 2. PD Control with Gravity Compensation

### Motivation
In a previous lecture, a standard PD controller was proposed: $u_i = K_{pi} \tilde{q}_i + K_{di} \dot{\tilde{q}}_i$. However, the premise behind that specific design was flawed. A small tweak to this PD controller—adding gravity compensation—guarantees that it works when the desired reference position $q^r$ is constant.

### System Dynamics and Control Law
The dynamics of the robot manipulator are given by:
$$ M(q)\ddot{q} + C(q,\dot{q})\dot{q} + K(q)\dot{q} + \nabla_q P = u $$
*(Note: $\nabla_q P$ represents the gravity vector $g(q)$, and $K(q)\dot{q}$ represents internal damping/friction).*

We define the position error as $\tilde{q} = q^r - q$. 
Since the reference $q^r$ is constant, $\dot{\tilde{q}} = -\dot{q}$ and $\ddot{\tilde{q}} = -\ddot{q}$.

**Control Law:**
$$ u = K_p \tilde{q} + K_d \dot{\tilde{q}} + \nabla_q P $$

**Closed-Loop Dynamics:**
Substituting the control law $(u)$ into the system dynamics, the gravity terms ($\nabla_q P$) cancel out, yielding the closed-loop equation:
$$ M(q)\ddot{q} + C(q,\dot{q})\dot{q} + K(q)\dot{q} = K_p \tilde{q} + K_d \dot{\tilde{q}} \quad \text{--- (Equation 3)} $$

---

## 3. Mathematical Preliminaries
To prove stability, we rely on the following matrix properties:
*   **(a)** The mass matrix $M(q) = M(q)^T$ is positive definite (p.d.).
*   **(b)** The damping matrix $K(q) = K(q)^T$ is positive semi-definite (p.s.d.).
*   **(c)** $\frac{\partial}{\partial x}(x^T P x) = 2x^T P$ 
    > **Correction/Addition:** This property is only strictly true if the matrix $P$ is **symmetric** ($P = P^T$). In this context, $K_p$ and $M(q)$ are symmetric, so the rule applies.
*   **(d) Passivity Property:** The matrix $\frac{d}{dt}M(q) - 2C(q,\dot{q})$ is **skew-symmetric**.

---

## 4. Stability Proof

**Theorem:** If $q^r$ is constant, the equilibrium point $(q, \dot{q}) = (q^r, 0)$ is Asymptotically Stable.

### Step 1: Define Lyapunov Candidate
The standard energy of a robot is $E(q,\dot{q}) = \frac{1}{2} \dot{q}^T M(q)\dot{q} + P(q)$. Inspired by this, we define our Lyapunov candidate using the kinetic energy and an artificial potential energy created by the controller error:
$$ V(q, \dot{q}) = \frac{1}{2}\dot{q}^T M(q) \dot{q} + \frac{1}{2} \tilde{q}^T K_p \tilde{q} $$

### Step 2: Show $V$ is Positive Definite
$V$ is positive definite at the equilibrium point $(q, \dot{q}) = (q^r, 0)$.
$$ V = 0 \iff \dot{q}^T M(q) \dot{q} = 0 \text{ and } \tilde{q}^T K_p \tilde{q} = 0 $$
Because $M(q)$ and $K_p$ are positive definite, this is only true iff $\dot{q} = 0$ and $\tilde{q} = 0$.

### Step 3: Calculate $\dot{V}$
Taking the time derivative of $V$:
$$ \dot{V} = \dot{q}^T M(q) \ddot{q} + \frac{1}{2}\dot{q}^T \dot{M}(q) \dot{q} + \tilde{q}^T K_p \dot{\tilde{q}} $$
Since $\dot{\tilde{q}} = -\dot{q}$, the last term becomes $-\dot{q}^T K_p \tilde{q}$. 

Now, substitute $M(q)\ddot{q}$ from Equation 3 ($M\ddot{q} = -C\dot{q} - K(q)\dot{q} + K_p\tilde{q} - K_d\dot{q}$):
$$ \dot{V} = \dot{q}^T \big[ -C\dot{q} - K(q)\dot{q} + K_p\tilde{q} - K_d\dot{q} \big] + \frac{1}{2}\dot{q}^T \dot{M} \dot{q} - \dot{q}^T K_p \tilde{q} $$

Distribute $\dot{q}^T$ and notice that the $K_p \tilde{q}$ terms cancel out ($\dot{q}^T K_p \tilde{q} - \dot{q}^T K_p \tilde{q} = 0$):
$$ \dot{V} = \frac{1}{2}\dot{q}^T (\dot{M} - 2C)\dot{q} - \dot{q}^T (K(q) + K_d)\dot{q} $$

### Step 4: Apply Skew-Symmetry
> **Lemma:** If $S \in \mathbb{R}^{n \times n}$ is a skew-symmetric matrix ($S^T = -S$), then $\forall x \in \mathbb{R}^n, x^T S x = 0$.
> **Proof:** Since $x^T S x$ is a scalar, it equals its transpose: $(x^T S x)^T = x^T S^T x$. Because $S$ is skew-symmetric, $S^T = -S$, meaning $x^T S^T x = -x^T S x$. Therefore, $x^T S x = -x^T S x \implies 2x^T S x = 0 \implies x^T S x = 0$.

From Preliminary (d), we know $(\dot{M} - 2C)$ is skew-symmetric. Applying the lemma:
$$ \frac{1}{2}\dot{q}^T (\dot{M} - 2C)\dot{q} = 0 $$

Leaving us with:
$$ \dot{V} = -\dot{q}^T (K(q) + K_d)\dot{q} \le 0 $$

> ⚠️ **Error Correction in Notes:** On page 3 of the handwritten notes, $\dot{V}$ is incorrectly written as $-\dot{q}^T(K_p + K(q))\dot{q} \le 0$. The $K_p$ term canceled out in the previous step. It must be $K_d$, the derivative gain, pulling energy out of the system.

### Step 5: Apply Krasovsky-LaSalle to Prove Asymptotic Stability
We must verify if $\dot{V} \equiv 0$ implies $(q, \dot{q}) = (q^r, 0)$.

Assume $K(q)$ is positive semi-definite and $K_d$ is positive definite. 
$$ \dot{V} = 0 \iff \dot{q}^T(K(q) + K_d)\dot{q} = 0 $$
This is true if and only if both $\dot{q}^T K(q) \dot{q} = 0$ and $\dot{q}^T K_d \dot{q} = 0$. 
Because $K_d$ is positive definite, $\dot{q}^T K_d \dot{q} = 0 \implies \dot{q} = 0$.

If $\dot{q}(t) = 0$ for all $t$, then the acceleration $\ddot{q}(t) = 0$ for all $t$.
Substitute $\dot{q} = 0$ and $\ddot{q} = 0$ back into the closed-loop dynamics (Equation 3):
$$ 0 = -C(0) - K(q)(0) + K_p \tilde{q} + K_d(0) $$
$$ \implies K_p \tilde{q} = 0 $$

Because $K_p$ is positive definite, it has no zero eigenvalues and is therefore invertible. Thus:
$$ K_p \tilde{q} = 0 \implies \tilde{q} = 0 $$

### Conclusion
We have shown that $\dot{q} \equiv 0 \implies \tilde{q} \equiv 0$. 
Therefore, $\dot{V} \equiv 0 \implies (q, \dot{q}) = (q^r, 0)$. 

By the Krasovsky-LaSalle theorem, the equilibrium point $(q, \dot{q}) = (q^r, 0)$ is **Asymptotically Stable (AS)**. $\blacksquare$