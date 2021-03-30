% This function gives the dynamics of the robot given the various matrices
% D, C, N
% th is a state vector having the joint angles and joint rates. This
% function returns the time derivative of the state vector (thd) that is then
% used by ODE45 to integrate the dynamic equations

function eqom=DeriveEOM(D,C,N,DOF,derive)
     syms t
     thd=sym('thd',[DOF 1]);
     th=sym('th',[DOF 1]);
     tau=sym('tau',[DOF 1]);
     eqom=[thd;inv(D)*(tau-C*thd-N)];
     eqom=simplify(expand(eqom));
     joint_state=[th;thd];
     if (derive == true) 
         matlabFunction(eqom,'file','GEN/ComputeEOM.m','vars',[{t},{joint_state},{tau}]);
     end
end