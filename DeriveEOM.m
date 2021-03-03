% This function gives the dynamics of the robot given the various matrices
% D, C, g
% th is a state vector having the joint angles and joint rates. This
% function returns the time derivative of the state vector (thd) that is then
% used by ODE45 to integrate the dynamic equations

function eom=DeriveEOM(D,C,G,DOF)
     syms t
     thd=sym('thd',[DOF 1]);
     th=sym('th',[DOF 1]);
     tau=sym('tau',[DOF 1]);
     syms eom;
     eom=[thd;inv(D)*(tau-C*thd-G)];
     eom=simplify(eom);
     joint_state=[th;thd];
     matlabFunction(eom,'file','GEN/ComputeEOM.m','vars',[{t},{joint_state},{tau}]);
end