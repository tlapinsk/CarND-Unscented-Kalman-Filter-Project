# Extended-Kalman-Filter
In this project, I used C++ to write a program that processes Lidar and Radar data to track / predict object positioning via an Unscented Kalman Filter. 

## Project Info
For a paper discussing the differences betweek EKFs versus UKFs, check out this [link](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.942.3499&rep=rep1&type=pdf).

To see the implementation please visit the following three files in the 'src' folder:

1. ukf.cpp
2. ukf.h
3. tools.cpp

## Setup instructions
FYI, I ran my code on a Macbook Pro. Please ensure you have downloaded the Udacity SDCND Simulator [here](https://github.com/udacity/self-driving-car-sim/releases/) and have installed cmake (3.5), make (4.1), and gcc/gcc+ (5.4).

1. Open Terminal
2. `git clone https://github.com/tlapinsk/CarND-Unscented-Kalman-Filter-Project.git`
3. `cd CarND-Unscented-Kalman-Filter-Project`
4. `sh install-mac.sh`
5. `mkdir build && cd build`
6. `cmake`
7. `make`
8. `./ExtendedKF`
9. Run the term2_sim application, select Project 1 & 2, and click 'Start'

## Results
My Extended Kalman Filter produced the below results. 'px' is the x-position, 'py' is the y-position, 'vx' is velocity in the x-direction, and 'vy' is velocity in the y-direction.

| Input |   MSE   |
| ----- | ------- |
|  px   | 0.0695  |
|  py   | 0.0830  |
|  vx   | 0.3397  |
|  vy   | 0.2220  |


![Visualization](https://github.com/tlapinsk/CarND-Unscented-Kalman-Filter-Project/blob/master/output/results.png?raw=true "Visualization")

## Resources
Most of my code is pulled from Udacity's introduction to Unscented Kalman Filters within the SDCND course material. Below are further resources and helpful links that I used to complete this project:

- [RMSE vx, vy too high](https://discussions.udacity.com/t/rmse-vx-vy-too-high/384143)
- [Simulation freezes](https://discussions.udacity.com/t/simulation-freezes-after-few-timestamps/375369/5)
- [Kalman Filters](https://medium.com/@kastsiukavets.alena/kalman-filter-extended-kalman-filter-unscented-kalman-filter-dbbd929f83c5)