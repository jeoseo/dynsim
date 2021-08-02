%Main executable for actual identification
clear all;
close all;

[t,js,tau]=BagToMatlab('js.bag'); %Replaces SimMB, we have real values now
InitBadParamsMB(); %I plug in what I "should" know about the robot, and represent the rest with symbolic
IdentifyMB(); %I refactor the dynamics equation to solve for the inertial parameters with LMS and the js, tau vectors
%Verification requires a second Bag file
VerifyRealMB();