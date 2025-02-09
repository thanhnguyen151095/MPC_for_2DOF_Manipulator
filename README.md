# MPC_for_Manipulator
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
\dot{x} = Ax + Bu
$$

where $x =  [q^T \dot{q}^T]^T$, A = 

The goal is to create an ideal control input $\tau$ that guides the actual joint position $q$ to match the target trajectory $q_d \in R^n$.

# Model Predictive Control (MPC) Formulation

