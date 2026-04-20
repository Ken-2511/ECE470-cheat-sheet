Here is a structured summary of the provided lecture notes. 

***Note from the teaching assistant:** I have carefully reviewed the mathematical derivations in these notes. I found a couple of logical errors/typos in the handwritten equations (specifically regarding an `argmax` that should be an `argmin`, and a missing `+` sign). I have explicitly pointed these out and corrected them in the summary below to ensure your formulas are mathematically sound.*

---

# Lecture Notes: Artificial Potential Field Method

**Context:** Continuation from the previous lecture covering the Artificial Potential Field method for motion planning.

## Step 1: Force Design (Repulsive Force)

### Assumptions and Obstacle Definitions
Assume the obstacle set $O$ is a convex set. If there are multiple obstacles, we model them as a union of multiple disjoint convex sets and use the superposition principle to sum their forces.

**Theorem (Projection onto a Convex Set):**
For any point $P \in \mathbb{R}^3$, there exists a unique closest point $\pi(P) \in O$ such that the distance $\|P - \pi(P)\|$ is minimized. 

> ⚠️ **CORRECTION:** The handwritten notes define this projection as $\pi(P) = \text{argmax}_{q \in O} \|P - q\|$. This is a **logical error**. To find the *closest* point on the obstacle to generate a repulsive force, you must minimize the distance, not maximize it. 
> **Corrected Equation:**
> $$ \pi(P) = \text{argmin}_{q \in O} \|P - q\| $$

*Example:* A spherical obstacle can be defined as $O = \{P \in \mathbb{R}^3 \mid \|P - c\| \leq R\}$, where $c$ is the center and $R$ is the radius.

### Repulsive Potential Function
Let $O_i^0$ be a control point on the robot. The repulsive potential $U_{i, \text{rep}}$ only acts when the robot is within a certain distance of influence, denoted as $\rho_0^i$. 

$$ 
U_{i, \text{rep}}(O_i^0) = 
\begin{cases} 
\frac{1}{2} \eta_i \left( \frac{1}{\|O_i^0 - \pi(O_i^0)\|} - \frac{1}{\rho_0^i} \right)^2 & \text{if } \|O_i^0 - \pi(O_i^0)\| \leq \rho_0^i \\ 
0 & \text{otherwise} 
\end{cases} 
$$
Where:
*   $\eta_i$ is a scaling/gain factor.
*   $\|O_i^0 - \pi(O_i^0)\|$ is the shortest distance from the robot point to the obstacle.

### Deriving the Repulsive Force
Force is the negative gradient of the potential function:
$$ F_{i, \text{rep}}(O_i^0) = -\nabla U_{i, \text{rep}} = - \left[ \frac{\partial U_{i, \text{rep}}}{\partial O_i^0} \right]^T $$

Applying the chain rule to take the derivative of the potential function yields:
$$ \frac{\partial U_{i, \text{rep}}}{\partial O_i^0} = \eta_i \underbrace{\left( \frac{1}{\|O_i^0 - \pi(O_i^0)\|} - \frac{1}{\rho_0^i} \right)}_{\text{From outer derivative}} \cdot \frac{-1}{\|O_i^0 - \pi(O_i^0)\|^2} \cdot \frac{(O_i^0 - \pi(O_i^0))^T}{\|O_i^0 - \pi(O_i^0)\|} \cdot \left( I_3 - \frac{\partial \pi}{\partial O_i^0} \right) $$

**Important Fact:** For a projection onto a convex boundary, the vector from the point to its projection is perpendicular to the boundary's tangent, meaning:
$$ (O_i^0 - \pi(O_i^0))^T \frac{\partial \pi}{\partial O_i^0} = \mathbf{0}_{1 \times 3} $$

Applying this fact and combining terms simplifies the expression. Taking the negative transpose gives the final formula for the repulsive force:

$$ 
F_{i, \text{rep}}(O_i^0) = 
\begin{cases} 
\eta_i \left( \frac{1}{\|O_i^0 - \pi(O_i^0)\|} - \frac{1}{\rho_0^i} \right) \frac{(O_i^0 - \pi(O_i^0))}{\|O_i^0 - \pi(O_i^0)\|^3} & \text{if } \|O_i^0 - \pi(O_i^0)\| \leq \rho_0^i \\ 
0 & \text{otherwise} 
\end{cases} 
$$

---

## Step 2: Gradient Descent (Motion Planning)

We map the workspace forces into the robot's joint space configuration $q$ to perform gradient descent.

### Define the Cost Function
The total potential (cost) is the sum of the attractive and repulsive potentials over all considered points $i$ on the robot:
$$ U(q) := \sum_{i=1}^n \Big( U_{i, \text{att}}(O_i^0(q)) + U_{i, \text{rep}}(O_i^0(q)) \Big) $$

### Gradient Descent Algorithm
*   **Initialize:** $q^0 = q_s$ (starting configuration)
*   **Iteration:** $q^{k+1} = q^k - \alpha_k \nabla_q U(q^k)$, where $\alpha_k$ is the step size/learning rate.

### Mapping Task-Space Force to Joint-Space Gradient
To find $\nabla_q U$, we apply the chain rule. Looking specifically at the attractive component:
$$ \nabla_q U_{i, \text{att}}(O_i^0(q)) = \left[ \frac{\partial}{\partial q} (U_{i, \text{att}}(O_i^0(q))) \right]^T $$
$$ = \left[ \frac{\partial U_{i, \text{att}}}{\partial O_i^0} \cdot \frac{\partial O_i^0}{\partial q} \right]^T = \left[ \frac{\partial O_i^0}{\partial q} \right]^T \cdot \underbrace{\left( \frac{\partial U_{i, \text{att}}}{\partial O_i^0} \right)^T}_{-F_{i, \text{att}}} $$
*(The exact same derivation applies to the repulsive component).*

### Final Update Rule
Substituting the mapped forces back into the gradient descent formula (and noting that subtracting a negative force results in a positive addition), we get the final update rule:

> ⚠️ **CORRECTION:** The handwritten notes appear to have a dot/comma between $F_{i, \text{att}}$ and $F_{i, \text{rep}}$ in the final equation. Because potentials are additive, the resulting forces must be summed together using a `+` sign.
> **Corrected Equation:**
> $$ q^{k+1} = q^k + \alpha_k \sum_{i=1}^n \left[ \frac{\partial O_i^0}{\partial q} \right]^T \Big( F_{i, \text{att}}(O_i^0(q^k)) + F_{i, \text{rep}}(O_i^0(q^k)) \Big) $$

### Calculating the Jacobian $\frac{\partial O_i^0}{\partial q}$
The term $\frac{\partial O_i^0}{\partial q}$ is the standard positional Jacobian matrix for point $i$. Because point $i$ is not affected by joints that come after it in the kinematic chain, the matrix is padded with zeros for joints $k > i$:
$$ \frac{\partial O_i^0}{\partial q} = \begin{bmatrix} J_{O_i}^1 & \cdots & J_{O_i}^i & \mathbf{0}_{3 \times (n-i)} \end{bmatrix} $$

Each column $j$ of the Jacobian depends on the joint type:
$$ 
J_{O_i}^j = 
\begin{cases} 
z_{j-1}^0 & \text{if Prismatic (P)} \\ 
z_{j-1}^0 \times (O_i^0 - O_{j-1}^0) & \text{if Revolute (R)} 
\end{cases} 
$$