# Mobile Manipulation Capstone
### Marcel Bonnici
### Northwestern University MS Robotics Student
Assignment: http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone
## Introduction
The python file plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down. The file creates two csv files, but only the `scene6.csv` file is to be simulated in V-REP.
## Explanation
A speed limit of 9 was implemented. Due to an inaccurate pickup location, I changed the wheels' radii to 1.5x their actual size to make the Jacobian pseudoinverse more convenient. Moreover, I initially programmed the end-effector trajectory entirely in Cartesian, but then made an exception between the first and second standoff positions with a ScrewTrajectory. This prevented collision between the wheels and box.

## Code Breakdown
The python file was drafted with three milestone sub-projects.

Milestone 1: youBot Kinematics Simulator and csv Output - Used to coordinate the functions/wheels

Milestone 2: Reference Trajectory Simulation - Generates trajectory of end-effector frame

Milestone 3: Feedforward Control - Uses a FeedbackControl function to calculate the kinematic task-space feedforward plus the feedback control law.

## Operation
Open a terminal in the under the project's directory and run `python final.py`.

## Result (Click Photo to Watch on YouTube)
The file `final.py` was run, and its generated `scene6.csv` file was thrown into V-REP to yield the glorious result below. [Click here for video.](https://www.youtube.com/watch?v=XvYdUPLrBC4)
[![VREP](https://i.ytimg.com/vi/XvYdUPLrBC4/maxresdefault.jpg)](https://youtu.be/XvYdUPLrBC4 "r2p3a")

### Error
Here's what the 'best' plotted errors looks like:
![Plotted errors](https://drive.google.com/uc?id=1fMIgSd7VRXlPBdki9X5qg7sQbVmQNFoQ)
