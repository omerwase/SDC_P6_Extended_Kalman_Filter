# SDC P6: Extended Kalman Filter


### Description:
This program implements an Extended Kalman Filter, with sensor fusion, that uses lidar and radar data to track a moving object. The program works in conjuction with [Udacity's SDC term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases). The simulator provides the program with simulated lidar and radar data. Using this data the program estimates the state of the object being tracked. The estimated state is compared with ground truth values to calculate the root mean squared error (RMSE).

---

### Dependencies:

* uWebSocket
* CMake
* Make

For details on installing dependencies see [Udacity's project repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

---

### Build Instructions:

1. Downloaded Udacity's SDC term 2 simulator.
2. Ensure the dependencies are isntalled.
3. Clone this repository and navigate to its local directory.
4. Create a build directory.
5. Initiate CMake from build directory.
6. Make the project.
```shell
$ mkdir build  
$ cmake ..  
$ make  
$ ./ExtendedKF  
```
7. Launch Udacity's SDC simulator and start project 1 simulation.
