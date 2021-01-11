# multilateration

Multilateration: the process of determining locations of points by measurement of distances from known points.

This C++ sample uses a nonlinear least squares solver from Ceres to solve for the multilateration problem.  
A Matlab script is also provided to generate some sample data to test with.

Trilateration: 3 beacons  
Multilateration: > 3 beacons  

See below for more on the multilateration problem:
https://en.wikipedia.org/wiki/Multilateration

Dependencies:
Ceres - http://ceres-solver.org/index.html  

Instructions:  
git clone https://github.com/hjamal3/multilateration.git  
cd multilateration  
mkdir build  
cd build  
cmake ..  
make  
./multilateration  
