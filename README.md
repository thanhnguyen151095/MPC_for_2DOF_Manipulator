# MPC_for_Manipulator
Model Predictive Tracking Control for Robot Manipulators

# Description of Robot Manipulator Dynamic Model
The dynamic model of a n-joint robot manipulator is expressed as:

$$
M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) = \tau
$$

where:  
- $ q \in R^n $ are the joint positions (angles).  
- $ M(q) \in R^{n\time n} $ is the **inertia matrix**.  
- $ C(q, \dot{q}) \in R^{n\time n} $  is the **Coriolis and centrifugal forces**.  
- $ G(q) \in R^{n} $ is the **gravity vector**.  
- $ \tau \in R^{n} $ is the **control torque**.  


