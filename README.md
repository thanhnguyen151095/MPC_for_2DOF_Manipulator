# Model Predictive Control (MPC) for Robot Manipulators
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

# MPC Formulation
We begin by defining the system state vector as follows:

$$
x = \begin{bmatrix} q^T \\ \dot{q}^T \end{bmatrix}^T
$$

Next, we can express the tracking error as:

$$
e = x - x_d
$$

Here, $x_d = [q^T_d \\ \dot{q}^T_d]^T$ denotes the desired trajectory, where $q_d$ and $\dot{q}_d$ represent the desired position and velocity, respectively.

MPC minimizes the following quadratic cost function over a prediction horizon \( N \):

$$
J = \sum_{k=0}^{N} \left( e ^T Q e + u_k^T R u_k \right)
$$

where:  
- $Q$ is the **state weighting matrix**.  
- $R$ is the **control weighting matrix**.  
- $N$ is the **prediction horizon**.  

The control input is optimized under the constraints:

$$
u_{\text{min}} \leq u_k \leq u_{\text{max}}
$$

MPC solves this optimization problem at every time step and applies only the first control input.

# PseudoCode

Define the number of steps N to predict ahead.

Define time step dt

1. Initial guess for control inputs u0 = zeros(n*N,1) % with n is number of joints
   
2. Define the cost function
   
  x_pred = x
  
  J = 0
  
  for k=1,...,N
  
    u_k = u(n*k-n+1:n*k)
    
    x_pred = x_pred + dt* $[\dot{q}, M^{-1}(q) \left( \tau - C(q, \dot{q}) \dot{q} - G(q) \right)]^T$
    
    e = x_pred - x_ref
    
    J = J + e' * Q * e + u_k' * R * u_k; % Compute cost
    
  end for

3.  Optimization options
4.  Solve the optimization problem
5.  Extract only the first control input
6.  Apply to robot manipulator
7.  Back to step 2

# Simulation Results and Discussion

**2-DOF Robot Manipulator**

<img src="https://github.com/user-attachments/assets/16ad4c61-c7ad-432b-a4a2-fa867f2ed659" alt="2-DOF Robot Manipulator" width="800">

**Simulation Video** 

<img src="Figures/video.gif" alt="Description" width="800">

**Simulation Results**

|_|_|
|:---:|:---:|
<img src="Figures/end-effector_tracking.png" alt="Description" width="600">| <img src="Figures/joint_tracking.png" alt="Description" width="600">|
<img src="Figures/e1.png" alt="Description" width="600">| <img src="Figures/e2.png" alt="Description" width="600">|
<img src="Figures/u1.png" alt="Description" width="600">| <img src="Figures/u2.png" alt="Description" width="600">|


# References
1. https://scholar.google.com/citations?user=Mc76hQoAAAAJ&hl=en


