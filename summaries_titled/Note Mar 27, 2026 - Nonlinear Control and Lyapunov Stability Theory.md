Here is a comprehensive and structured summary of the lecture notes. 

***

# ECE470: Robotics - Nonlinear Control and Stability

## 1. Stability Definitions
Consider a dynamical system $\dot{x} = f(x)$ with an equilibrium point $\bar{x}$.
*   **Stable:** $\forall \epsilon > 0, \exists \delta > 0$ such that $\|x(0) - \bar{x}\| < \delta \implies \|x(t) - \bar{x}\| < \epsilon \quad \forall t \ge 0$.
*   **Asymptotically Stable (AS):** The system is stable *and* converges to the equilibrium point: 
    $\exists \delta > 0$ such that $\|x(0) - \bar{x}\| < \delta \implies x(t) \to \bar{x}$ as $t \to \infty$.

*(The notes also contain sketches illustrating positive definite (p.d.) and positive semi-definite (p.s.d.) functions. A p.d. function is zero only at the origin and strictly positive elsewhere, forming a "bowl" shape. A p.s.d. function is non-negative everywhere but can be zero at points other than the origin).*

## 2. Lyapunov's Stability Theorem
Let $\bar{x}$ be an equilibrium point of the system $\dot{x} = f(x)$. 
Suppose there exists a continuously differentiable ($C^1$) function $V: \mathbb{R}^n \to \mathbb{R}$ such that:
1.  $V$ is **positive definite (p.d.)** at $\bar{x}$. *(Think of this as an "energy-like" function).*
2.  $\dot{V} = \frac{\partial V}{\partial x}f(x)$ is **negative semi-definite (n.s.d.)**. *(Meaning energy is non-increasing).*

**Conclusion:** Then $\bar{x}$ is **stable**.
*   **(ii') Stronger Condition:** If instead of (ii), $\dot{V}$ is **negative definite (n.d.)** at $\bar{x}$ (strictly decreasing), then $\bar{x}$ is **Asymptotically Stable (AS)**.

> **TA Correction & Clarification on Page 2 Example:**
> On the top of page 2, the notes introduce $V = \frac{1}{2}(x_1^2 + x_1x_2 + x_2^2)$ and claim its derivative is $\dot{V} = -\frac{1}{2}(x_1^2 + x_1x_2 + x_2^2) = -V$. 
> *Note that this is an abstract, generic mathematical example showing what a "perfect" Lyapunov function for asymptotic stability might look like. This specific math does **not** apply to the mass-spring-damper system introduced on page 1. The actual Lyapunov analysis for the mass-spring-damper system is done later on page 3 using physical energy.*

## 3. Quadratic Forms and Definiteness
A quadratic function takes the form $V(x) = x^T P x + C^T x + D$. A pure **Quadratic Form** is:
$$V(x) = x^T P x$$
where $P = P^T \in \mathbb{R}^{n \times n}$ (a symmetric matrix) and $x \in \mathbb{R}^{n \times 1}$.

**Definition:** We say the matrix $P = P^T$ is positive definite (p.d.) if the function $V(x) = x^T P x$ is p.d. at $x = 0$.

### Criteria for a Matrix $P$ to be Positive Definite
*   $P$ is p.d. $\iff$ all eigenvalues of $P$ are $> 0$.
*   $P$ is p.s.d. $\iff$ all eigenvalues of $P$ are $\ge 0$.
*   **Sylvester's Criterion:** $P$ is p.d. $\iff$ all **principal leading minors** of $P$ are $> 0$.
    *(Note: The handwritten notes mistakenly write "minus". The correct mathematical term is "minors").*
    
    The principal leading minors are defined as the determinants of the upper-left sub-matrices:
    $$M_1 = P_{11}$$
    $$M_2 = \det \begin{bmatrix} P_{11} & P_{12} \\ P_{21} & P_{22} \end{bmatrix}$$
    $$...$$
    $$M_n = \det(P)$$

### Examples of Sylvester's Criterion
**Example 1:**
$$P = \begin{bmatrix} 1 & 1/2 \\ 1/2 & 1 \end{bmatrix}$$
*   $M_1 = 1 > 0$
*   $M_2 = \det(P) = (1)(1) - (1/2)(1/2) = 1 - 1/4 = 3/4 > 0$
*   **Result:** $P$ is positive definite.

**Example 2:**
$$P = \begin{bmatrix} 2 & 1 & 0 \\ 1 & 3 & -1 \\ 0 & -1 & 2 \end{bmatrix}$$
*   $M_1 = 2 > 0$
*   $M_2 = \det \begin{bmatrix} 2 & 1 \\ 1 & 3 \end{bmatrix} = (2)(3) - (1)(1) = 5 > 0$
*   The notes state: $M_3 = \det(P) < 0 \implies P \text{ is not p.d.}$

> **⚠️ MAJOR TA CORRECTION:**
> The notes contain a calculation error for $M_3$. Let's calculate the determinant of the $3 \times 3$ matrix correctly:
> $$M_3 = 2 \det \begin{bmatrix} 3 & -1 \\ -1 & 2 \end{bmatrix} - 1 \det \begin{bmatrix} 1 & -1 \\ 0 & 2 \end{bmatrix} + 0$$
> $$M_3 = 2((3)(2) - (-1)(-1)) - 1((1)(2) - 0) = 2(6 - 1) - 1(2) = 2(5) - 2 = 8$$
> Since $M_3 = 8 > 0$, and all previous minors are positive, **$P$ IS ACTUALLY POSITIVE DEFINITE.** 

## 4. Mass-Spring-Damper System (Physical Energy Example)
Consider the system $m\ddot{y} = -ky - b\dot{y}$. Let state variables be $x_1 = y$ and $x_2 = \dot{y}$. 
The state-space equations are:
$$ \begin{aligned} \dot{x}_1 &= x_2 \\ \dot{x}_2 &= -\frac{k}{m}x_1 - \frac{b}{m}x_2 \end{aligned} $$
Assume $k, m, b > 0$. The only equilibrium point is $\bar{x} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$.

**Step 1: Define Lyapunov Candidate (Mechanical Energy)**
$$E = \text{Kinetic} + \text{Potential} = \frac{1}{2}mx_2^2 + \frac{1}{2}kx_1^2$$
Matrix form: 
$$E = \frac{1}{2} x^T \begin{bmatrix} k & 0 \\ 0 & m \end{bmatrix} x$$
Checking minors: $M_1 = k > 0$ and $M_2 = km > 0$. Therefore, $E$ is p.d. at $x = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$.

**Step 2: Take the Derivative**
$$ \begin{aligned} \dot{V} &= \frac{\partial V}{\partial x_1}\dot{x}_1 + \frac{\partial V}{\partial x_2}\dot{x}_2 \\ &= (kx_1)(x_2) + (mx_2)\left(-\frac{k}{m}x_1 - \frac{b}{m}x_2\right) \\ &= kx_1x_2 - kx_1x_2 - bx_2^2 \\ &= -bx_2^2 \end{aligned} $$
Since $\dot{V} = -bx_2^2 \le 0$, it is **negative semi-definite** (it is 0 whenever velocity $x_2 = 0$, regardless of position $x_1$). 
By Lyapunov's Theorem, the origin is **Stable**.

**Intuition for Asymptotic Stability:**
The phase portrait shows a spiral converging to the origin. 
*   We know energy is dissipated as long as $x_2 \neq 0$ (speed is nonzero).
*   Intuitively, $x_2(t) \to 0$.
*   In steady state, $x_2(t) \equiv 0 \implies \dot{x}_2(t) \equiv 0$.
*   Plugging this into the system equations: $0 = -\frac{k}{m}x_1 - \frac{b}{m}(0) \implies x_1(t) \equiv 0$.
*   Therefore, the system must converge to $x = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$. This logic is formalized by the Invariance Principle.

## 5. Krasovsky-LaSalle Invariance Principle
This principle allows us to prove Asymptotic Stability even when $\dot{V}$ is only negative semi-definite.

Suppose $\bar{x}$ is an equilibrium of $\dot{x} = f(x)$ and there exists a $C^1$ function $V$ that is positive definite at $\bar{x}$ such that $\dot{V} = \frac{\partial V}{\partial x}f(x) \le 0$.
*   Then $\bar{x}$ is **stable** and $\dot{V}(x(t)) \to 0$ as $t \to \infty$.
*   **Moreover:** If the only trajectory where $\dot{V}(x(t)) \equiv 0 \quad \forall t$ is the trivial trajectory $x(t) \equiv \bar{x}$, then $\bar{x}$ is **Asymptotically Stable (AS)**.

## 6. Historical Context
*(Compiled from notes and chalkboard image)*
*   **1892:** A.M. Lyapunov publishes his foundational work on stability.
*   **1930s:** N.G. Chetayev applies Lyapunov's methods to aeronautical problems.
*   **1930s+:** Aleksandr Andronov applies nonlinear dynamics to radio engineering.
*   **1940s - 1960s:** An "explosion of work" occurs, moving from Russian literature into the USA and global academia, foundational to modern control theory.