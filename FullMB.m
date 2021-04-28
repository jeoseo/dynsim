clear all;
close all;
InitParamsMB(); %I plug in all known robot parameters
SimMB(); %I simulate the robot with some trajectory to find a torque tau
InitBadParamsMB(); %I plug in what I "should" know about the robot, and represent the rest with symbolic
IdentifyMB(); %I refactor the dynamics equation to solve for the inertial parameters with LMS and the js, tau vectors
VerifyMB(); %I plug the inertial parameters back into my dynamics equation, and resimulate the robot with the same trajectory
            %finding some torque tau, and compare it to the original
