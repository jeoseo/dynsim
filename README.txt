This is going to mostly be a copy of the code Simaan sent me, but I figure it's a good idea for me to come up with my own so I can learn.

In general:
Derive=function that symbolically generates another function to compute some expression
Compute=function intended on using the derived expression and real numbers to create some numeric result
Sim=Dynamically simulates and visualizes some robot (executable)
Identify=Identify some robot (executable)
-SimMB simulates the manufacturing base robot
-SimMLS simulates the MLS textbook eaxmple 4.3 robot
-SimTLP simulates a simple two link planar manipulator
-IdentifyTLP identifies viscous friction and the inertia tensor, for a noiseless
measurement (this is the easy part). Run after running SimTLP
-IdentifyMLS identifies viscous friction and the inertia tensor, for a 
measurement with gaussian noise (doesn't really work). Run after running SimMLS
-ViewSim(t,js,record) views a simulated robot, where t and js represent the jointstate and timestamps,
and record is a boolean designating whether or not to generate a video.