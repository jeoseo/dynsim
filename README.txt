This code includes functions and scripts to identify the current torque relationship of the Dynamixel
motors and the dynamic parameters of the robot

In general:
Derive=function that symbolically generates another function to compute some expression
Compute=function intended on using the derived expression and real numbers to create some numeric result
Sim=Simulates the dynamics of some robot, or is working with a simulated robot 
Identify=Identify some robot or relationship 
FindGood=Uses the surrogate optimizer to find an optimal set of parameters for some function
Full=Runs multiple other scripts, executable
 
Important Files:
FindGoodTorqueArm - used to find parameters for the current-torque of a Dynamixel given a rosbag of joint states
FindGoodTraj - used to find the parameters for the excitation trajectory of some robot
FullSimMB - Identifies the parameters of a simulated manuafacturing robot
FullMB - Identifies the parameters of the real manufacturing robot (needs rosbag, unfinished)
FullComputedTorqueJoint4 - Produces figures for the computed torque trajectory run with just joint 4

FullTTA - Used to graph results of a current-torque relationship of a Dynamixel given a certain trajectory
ViewSim(t,js,record) - Views a simulated robot, where t and js represent the jointstate and timestamps,
and record is a boolean designating whether or not to generate a video. Use to verify a what a trajectory would actually do