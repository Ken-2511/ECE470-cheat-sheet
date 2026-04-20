Here is a well-structured Markdown summary of the provided lecture notes. 

# ECE470: Robotics Lecture Notes Summary

## Schematically Representing a Robot

Robots are physically constructed by interconnecting rigid bodies (links) via joints. 

### Basic Joint Types
There are two fundamental types of joints used to describe kinematic chains:
1. **Revolute Joint (R):** Allows rotational motion between two links. The joint variable is an angle, typically denoted as $\theta_i$.
2. **Prismatic Joint (P):** Allows translational (sliding) motion between two links. The joint variable is a linear displacement, typically denoted as $d_i$.

### Complex Joints
More complex joints can be mathematically and schematically represented using combinations of the two basic types. 
*   **Example (Spherical Joint):** A spherical (ball-and-socket) joint allows rotation in three dimensions. It is kinematically equivalent to three intersecting revolute joints, denoted as an **RRR** structure, with joint variables $\theta_1, \theta_2,$ and $\theta_3$.

### Joint Vector / Configuration Vector
The configuration of a robot at any given time is fully described by a vector containing all of its joint variables.

$$q = (q_1, q_2, \dots, q_n)$$

Where each $q_i$ is either $\theta_i$ (if joint $i$ is revolute) or $d_i$ (if joint $i$ is prismatic). 
*   **Degrees of Freedom (DoF):** For a standard serial robot, the number of Degrees of Freedom is equal to the total number of joints: 
    $$n = \text{total number of joints} = \text{DoF}$$

---

## Kinematic Structures

**Kinematic Structure** refers to the specific way in which joints and links are strung together to form the robot. There are three typical kinematic structures:

1. **Serial Chains:** A single, continuous sequence of links and joints from the base to the end-effector. 
    *   *Example:* An articulated manipulator (e.g., standard robotic arm).
2. **Parallel Chains:** Structures that contain closed kinematic loops, where multiple serial chains connect a single base to a single end-effector platform. 
    *   *Example:* Hexapod (Stewart platform).
3. **Tree-like Structures:** Structures that branch out from a central base or torso into multiple open chains.
    *   *Example:* A humanoid robot (e.g., Robot "Lola" which has 24 total DoF distributed across the head, arms, and legs).

---

## Rigid Motions & Coordinate Frames

To understand the position and orientation of the entire robot in 3D space, we assign coordinate frames to the base and to the individual links. 

To analyze this mathematically, we need to understand:
1. What a coordinate frame is.
2. How to represent the same physical object or point in different coordinate frames.

### Defining a Coordinate Frame
A coordinate frame attached to a link $i$ is denoted as $O_i x_i y_i z_i$. 
*   $O_i$ is a point representing the **origin** of the frame.
*   $x_i, y_i, z_i$ are mutually orthogonal **unit vectors** representing the axes.

### Handedness of Frames
Coordinate frames can theoretically be left-handed or right-handed. However, in robotics, we strictly use **Right-Handed Frames**.

*   **Right-Hand Rule:** A coordinate frame $(x_i, y_i, z_i)$ satisfies the right-hand rule if pointing your right index finger along $x_i$ and your middle finger along $y_i$ causes your thumb to point along $z_i$. 

*(Instructor/TA Addition for completeness)*: Mathematically, a right-handed frame must satisfy the cross-product relationship:
$$x_i \times y_i = z_i$$