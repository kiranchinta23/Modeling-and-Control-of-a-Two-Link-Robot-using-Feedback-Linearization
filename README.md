# Modeling-and-Control-of-a-Two-Link-Robot-using-Feedback-Linearization

The goal of this project is to simulate a two‐link robotic manipulator model and develop a tracking controller for it using the feedback linearization method to track desired trajectories in the θ1 -θ2 and x‐y coordinates.

Figure from below shows the diagram of the two‐link. Each link is controlled by a separate actuator (motor), and the rotation of each link is measured via a separate sensor (encoder). The list of parameters for the dynamic model and their values for the simulation are provided in Table 1.

![link](https://user-images.githubusercontent.com/45347225/52861875-9e6cff00-30e8-11e9-8f66-96015769cc9d.PNG)
![table1](https://user-images.githubusercontent.com/45347225/52861856-94e39700-30e8-11e9-8b73-6faa8fbaafa1.PNG)

The mathematical dynamic model for a general robotic manipulator is given by:
![img](https://user-images.githubusercontent.com/45347225/52862775-e8ef7b00-30ea-11e9-87ec-c6270bf22a34.PNG)
where M is the inertia matrix; C contains centripetal, Coriolis, and viscous damping effects, N represents
the gravitational torques, and  τ is the control torque vector.
