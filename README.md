# A Distributed Approach to Interactive Surveillance Using a UAV Mounted Camera Network

Multi-Camera surveillance systems have become a modern trend in many fields both for research and for the industry. These systems are being increasingly used to enhance safety and security in public places such as parks, airports, and banks, as well as in restricted areas like government and military facilities. Furthermore, the possibility to mount cameras on Unmanned Aerial Vehicles (UAVs) extends the capabilities of these surveillance systems to a whole new level. Indeed, the employment of flying cameras allow for a substantial reconfigurability of the network and enable for new perspectives on the environment and wider coverage.

### Project Abstract
The aim of this project is to find a possible solution to the Interactive Surveillance problem using multiple UAV mounted cameras, from a top down perspective, in known environments. The coordinated patrolling problem is approached in a distributed fashion using a Bayesian-based Greedy algorithm with State Exchange. The results of this strategy are compared in restricted environments with the optimal paths obtained through linear programming. Furthermore, the overall performance is evaluated on large scale environments, where the optimal problem becomes infeasible. Kalman’s theory is employed to implement a smart target tracking algorithm with camera zoom control optimized for target containment and information loss minimization. Several simulations are performed tracking a target with different trajectory models and varying the sampling frequency of the filter and the accuracy of the detection. Finally, the robustness of such an algorithm to measurement errors and camera failures is put to the test.

### Project Organization
```
.
├── data/                                       : Contains all coins images
├── src/                                        : Contains report images
├── CMakeLists.txt                              : CMake instructions list
├── coins.cpp                                   : Main routine
├── coins_toolbox.cpp                           : Coin class (/w Classification Code)
├── coins_toolbox.hpp                           : Coin class Header file
└── README.md                                   : Project Report 
```
