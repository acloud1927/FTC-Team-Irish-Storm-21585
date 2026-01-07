This code is for the team 21585 Irish Storm robot, which is built as follows:

The robot has 4 mechanum wheel powered by goBuilda motors, and uses goBuilda Odometry Pods to control movements in Auto.

The robot has front motor which powers the intake and the transport wheels, along with a dedicated shooting motor at the top back of the robot.

The robot has 2 linear slided powered by goBilda motors, with a locking mechanism. These raise the robot and lock it in even without power.

The robot uses a webcam attached on the shooting side to read AprilTags.

In autonomous, this allows the robot to move based on distance and turn based on degrees. 
The shooting motor is set to launch 3 balls in sequence at a consistent rpm.

In teleop, the mechanum wheels provide agility and speed, and an equation has been generated that should allow the robot to shoot at the right power 
to always make a shot if it can see the AprilTag. This was done by making a linear regression after mapping points for distance from the AprilTag (ftcPose.y) and shooting rpm.

