# Controlling 
***
#### To make the testing of our Control algorithms easier, we considered a spherical ball representing holonomic degrees of freedom for our bot.

The State matrix can be expressed as 

![State Matrix](https://github.com/astitva22/RoboReG-Gyrosphere/blob/astitva/assets/State_matrix.png)

#### Dynamics of the simplified model

![State Equation](https://github.com/astitva22/RoboReG-Gyrosphere/blob/astitva/assets/State_eqn_expanded.png)

Now to get the system to converge to a specific state, we have various control methods such as- 

1. Proportional-Intergral-Derivative Control
2. Linear Quadratic Regulator
3. Model Predictive Control

For our simplified model, we have used Linear Quadratic Regulator or LQR controller. This is because it has robustness and ease of implementation and our model is already linear.

For final testing of the robot design, we will be using Model Predictive Control, and work is in progress for that.

#### Linear Quadratic Regulator

Considering the placement of our actuators and Dynamics of the system, The State Equation will look like

![State Equation with values](https://github.com/astitva22/RoboReG-Gyrosphere/blob/astitva/assets/state_eqn_numeric.png)

The cost Matrix for State Q and the cost Matrix for Actuators R were experimently observed to be 

![Cost matrices](https://github.com/astitva22/RoboReG-Gyrosphere/blob/astitva/assets/cost_matrices.png)

Feeding these to the  controlller_LQR function from controlpy library, we get the bestfitting Eigen value Matrix K for the system to converge to fed the target value.

![control equation]()

by setting the value of X_bar, we can make the system(or bot) to our desired state.

## Following a Mathematical curve 

To make the bot follow a nathematical curve, we can simpily pass the desired equation (preferably passing through origin), split it into various points and then updating the next target point once the bot has converged to the one immediately before it. 

Following piece of code in [curve_following_pos-control.py](https://github.com/astitva22/RoboReG-Gyrosphere/blob/final_branch/kinematicsSim/curve_following_pos-control.py) takes care of it

```
if(disp < 0.1):
    target_x += 0.5                         # updating the next state
    target_y = target_function(target_x)    # y as given function of x
```

Here we took the step for x = 0.5 as it gives decent accuracy while following given curve.

# Results
***
## Achieved Objectives 
### A fully functional urdf file for our imporvised mechanism. 
The designed urdf file is completely ready and can be used for Simulation in pybullet or in any other simulation software with a few tweaks.
* testing with the base of the bot fixed
    ![fixed base](https://github.com/astitva22/RoboReG-Gyrosphere/blob/astitva/assets/urdf_fixed-base.webm)
* Video for mobile operation can be found [here](https://github.com/astitva22/RoboReG-Gyrosphere/blob/astitva/assets/urdf_mobile.webm). Though there is no proper controller for it yet.

### Linear Qudratic Regulator for accurate path following
We Have designed two controllers for Path following using LQR control.
* Path following using the current position as the state vector. Video is attached below.
* Path following using the current velocity as the state vector. Needs a bit more work.

#### ![Curve Following using position control](https://github.com/astitva22/RoboReG-Gyrosphere/blob/astitva/assets/curve_following.webm)

# References
***
[Playlist on Controls by Steve Brunton](https://youtu.be/1_UobILf3cc)


