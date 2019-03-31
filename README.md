# cgn_test
## Description
The program is intended for V-REP simulator, where simple Ackermann car is following a given trajectory (2D points).     
There are 6 modules:  
- drivableRobot.h and ackermannCar.h/.cpp: abstract class for drivable robots with wheels which is able to steer to given angle and drive by given speed; and concrete class for ackermann car
- pathFollower.h/.cpp: class which verifies the given trajectory and calculates steering angle for it
- trajVisualizer.h/.cpp: class which visualizes given trajectory and given two points
- main.cpp: class Application which connects all parts, reads given trajectory from file and check it
- pid.h/.cpp: class which realizes pid functions, but is not used in this project
- scene folder: contains a scene with the simple ackermann car and a map
## How to use
1. ``` git clone https://github.com/igzubov/cgn_test/ && cd cgn_test```
2. ``` mkdir build && cd build ```
3. Change line 5 of CMakeLists.txt from     
``` set(V-REP_PATH /home/igor/cgn/V-REP_PRO_EDU_V3_6_1_Ubuntu16_04) ```    
to    
``` set(V-REP_PATH your_path/V-REP_PRO_EDU_V3_6_1_Ubuntu16_04) ```    
where **your_path** is an absolute path to your V-REP simulator directory    
4. ``` cmake .. && make ```
5. Place points file in the folder with executable **cgn_vrep**
6. Run scene in the V-REP
7. ``` ./cgn_vrep points_file ```, 
where **points_file** is a file containing points in the next format:  
number_of_points    
p1_x p1_y    
p2_x p2_y    
... ...

The number of points must be **at least 2** points, which are in the map range (default [-48.5, 48.5] for both x and y)    
For example use points.txt from this repository, which gives spiral trajectory    

![Spiral trajectory](https://i.imgur.com/JA12Ex1.png)
