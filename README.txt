This is the code that was used in ENED 1120 Project 5.

The project 5 assignment description is included in the repository.

A video of the final robot performing the tasks can be watched here:
https://youtu.be/5SFVOaWzkVg

The robot uses a PID algorithm to drive straight and tracks the rotation of the left motor to measure the distance it has traveled. 
Before searching for a box, it will navigate to an intersection point between the 2 closes aisles. From there it will navigate closer to the boxes with more precision to properly read them.
Once a box is found, the robot takes a continual reading of the reflected light intensity off of the "barcode" and analyzes the data it generates to determine if it is the correct box.
Once the correct box is found, the robot will rotate to lift it and then bring it back to its home.

Due to the imprecision of the Lego robotics kit, every instance the robot moves it will first move quickly to get where it needs to go, then move slowly in the opposite direciton to account for overshooting.

Python was used because it was the most efficient and effective option. 