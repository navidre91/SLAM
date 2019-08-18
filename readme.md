# SLAM

Assume that you have a lost robot in a field and it does not have any idea that where it is located. There are a few beacons in the field that the robot can massure its distance from them whenever it gets close enough to them. However the measurements are noisy. The robot has a list that contains the exact position of the beacons, but it has no idea which measurement is associated with wich beacon. The robot decides to go around and collect the measurements from beacons. Also the robots can have a noisy esimate of how far and in what direction is has travelled. This project is about the implementation of Kalman filtering in order to make the best use of the data in order to: 
    1- Have the best guess of label for the beacon measurements 
    2- Find the best estimate of the robot's location in the field. 
