%Main executable for the identifying parameters for a simulation of the manufacturing robot

%ipm=identified parameters map, keys gives the names and values gives the
%values
%rms_tau,rms_eff give the rms errors of each respectively
clear all;
close all;
rng(2);
InitSimParamsMB(); %I plug in all known robot parameters
SimGeneric(); %I simulate the robot with some trajectory to find a torque tau
InitUnknownParamsMB(); %I plug in what I "should" know about the robot, and represent the rest with symbolic
IdentifyMB(); %I refactor the dynamics equation to solve for the inertial parameters with LMS and the js, tau vectors
VerifyMB(); %I plug the inertial parameters back into my dynamics equation, and resimulate the robot with the same trajectory
            %finding some torque tau, and compare it to the original
InitUnknownParamsMB();
CompareParams(); %A second verification script: produces a table comparing the original and identified parameters (only possible in simulation)
%SimComputedTorqueMB(); % this runs a computed torque controller on the
%manufacturing base robot