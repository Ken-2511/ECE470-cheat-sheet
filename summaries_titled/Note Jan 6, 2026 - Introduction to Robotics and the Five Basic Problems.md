Here is a structured Markdown summary of the lecture notes.

# ECE470: Robotics
**Professor:** Prof. Manf. Maggiore

**Textbook:** Spong - Hutchinson - Vidyasagar, 2020 edition
*   *Note:* This is considered a very standard book in the robotics industry.

---

## Course Components & Logistics

**1. Tutorials**
*   Start this Friday, Jan 9.
*   Considered an integral part of the course.

**2. Quizzes**
*   First quiz during the tutorial on Feb 6.

**3. Midterm Exam**
*   Thursday, March $5^{\text{th}}$, 6:00 PM - 8:00 PM.

**4. Laboratory (Labs)**
*   5 labs total, done in groups of 2-3 students maximum.
*   **Lab 0:** Mandatory, but not graded.
*   **Labs 1-4:** Require a preparation submission and a code submission.
    *   *Prep:* Submitted 48 hours in advance.
    *   *Code:* Submitted no later than one week after the lab.
    *   *Task Example:* Use a robot to draw something on paper.

**5. Attendance & Quercus**
*   Attendance is not graded, but it informs follow-ups.
*   **Quercus Navigation:** Quercus $\rightarrow$ Quizzes $\rightarrow$ Jan 6 $\rightarrow$ code 1234 $\rightarrow$ student number.

---

## What is this course about?
The core focus of this course is **algorithms making robots move.** This is broken down into five basic problems.

### The Five Basic Problems

**1. Forward Kinematics**
*   **Definition:** From known joint angles/displacements (obtained via sensor measurement), determine the roto-translation of the end effector.
*   *Roto-translation* refers to the endpoint position and rotation in space.

**2. Inverse Kinematics**
*   **Definition:** From the desired roto-translation of the end effector, reverse-engineer the required joint variables (angles/displacements).

**3. Dynamics**
*   **Concept:** Understanding the equations of motion of robots using Lagrangian modelling.
*   *Diagram Note:* A 2-link planar arm is shown with joints $1$ (base) and $2$ (elbow).
*   *Physical Behavior:* If joint $1$ is fixed and joint $2$ moves, it brings significant torque back to joint $1$. *(Instructor's Note: This illustrates "dynamic coupling" between links, where the motion of one link generates reaction forces/torques on other links).*

**4. Motion Planning**
*   **Concept:** Planning a collision-free path for the robot to complete a task.
*   *Diagram Note:* A robotic arm is shown planning a trajectory to move an object from position [A] to position [B] by maneuvering over an obstacle.

**5. Control**
*   **Concept:** Developing **adaptive control** methods.
*   **Goal:** Being able to control the robot without explicitly knowing its exact physical parameters (e.g., moment of inertia, gravity effects, etc.).

---

## Course Policies & Closing Remarks

*   **Lecture Materials:** The professor will *not* post lecture materials online. Students are expected to come to the lecture.
*   **Missed Lectures:** If you miss a lecture, email the professor to ask for notes.
*   **The Deal/Promise:** You will not regret taking this course, and you will not forget it for many years.