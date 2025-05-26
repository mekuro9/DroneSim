# DroneSim
Steps to compile and run cargo-drone simulation:
------------------------------------------------
1. Unzip in an empty directory <drone_ws/src>
2. Go to <drone_ws> (not in src)
3. colcon build
5. In new terminal:
cd <drone_ws>
. install/local_setup.sh
ros2 run dronesim talker
(option to choose 1 -> for Drone with Cargo and 0-> for Drone without Cargo
6. In another new terminal:
cd <drone_ws>
. install/local_setup.sh
ros2 topic pub inputs std_msgs/Float32MultiArray "data: [5, -0.1]"
    (or any other value)

if in Step 5, option is 1 then follow 7:
7. In another new terminal:
cd <drone_ws>
. install/local_setup.sh
ros2 topic pub method std_msgs/Bool "data: true"
(or false)

true : for runge kutta method
false: for euler method

To exit from terminal Ctrl+C
