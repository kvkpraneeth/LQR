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
 
  In general, LQR is built for maintaining a value, which is not what a controller is. But in order to use it as a controller, consider the system error to be the system state and update the error every iteration. This will make LQR regulate the error at 0 while minimizing the quadratic cost, which makes it a perfectly good controller.
  
  #### Test 2 : Video
  
  

https://user-images.githubusercontent.com/55596533/127761370-56568959-60c7-47f2-ad2e-d3ec1033253a.mp4


