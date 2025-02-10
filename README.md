# MPC for Robot Manipulators
Model Predictive Tracking Control for Robot Manipulators

# Description of Robot Manipulator Dynamic Model
The dynamic model of a n-joint robot manipulator is expressed by the following equation:

$$
M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) = \tau
$$

where:  
- $q \in R^n$ are the joint positions (angles).  
- $M(q) \in R^{n \times n}$ is the **inertia matrix**.  
- $C(q, \dot{q}) \in R^{n \times n}$  is the **Coriolis and centrifugal forces**.  
- $G(q) \in R^{n}$ is the **gravity vector**.  
- $\tau \in R^{n}$ is the **control torque**.  

Thus, the dynamic equation of the robot manipulator can be reformulated as follows:

$$
\dot{q} = M^{-1}(q) \left( \tau - C(q, \dot{q}) \dot{q} - G(q) \right)
$$

The goal is to design an optimal control input $\tau$ that guides the actual joint position $q$ to match the target trajectory $q_d \in R^n$.

# Model Predictive Control (MPC) Formulation
We begin by defining the system state vector as follows:

$$
x = \begin{bmatrix} q^T \\ \dot{q}^T \end{bmatrix}^T
$$

Next, we can express the tracking error as:

$$
e = x - x_d
$$

Here, $x_d = [q^T_d \\ \dot{q}^T_d]^T$ denotes the desired trajectory, where $q_d$ and $\dot{q}_d$ represent the desired position and velocity, respectively.


