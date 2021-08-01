# Linear Quadratic Regulator

## About Repo:

  This repo provides a C++ Library for implementation of LQR.
  
## How to use this Repo:

  This repo was made with ROS2 in mind, so colcon build tool is required and ROS2 standard workspace structure should be used while building the package. </n>
  
## Validity of the code:

  The code and math validity can be confirmed by the test examples implemented.
  
  ### Test 1 : LQR Problem from Christopher lum's video; Spring Mass State Regulation Problem:
  
  [Solution](https://www.youtube.com/watch?v=wEevt2a4SKI&t=3656s)
  
  ### Test 2 : Joint Control using LQR based on Error Regulation:
  
  This solution was displayed in CoppeliaSim. In order to check that, you need to add Float32MultiArray message to the meta of the ros2 package. For more information: [A good starting point](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm)
 
  LQR as the name suggests is a regulator, maintains a value or state while trying to minimize a quadratic cost function that is based upon the effort it can apply on the actuators as well as the error importance of the state (R and Q respectively). 
  
  For reaching a reference state with LQR, we have to regulate the error to a 0. Which implies that instead of considering the state vector to be system state, the state vector should be the error. 

  #### Test 2 : Video

https://user-images.githubusercontent.com/55596533/127761370-56568959-60c7-47f2-ad2e-d3ec1033253a.mp4


