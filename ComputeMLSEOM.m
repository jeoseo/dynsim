% This function gives the dynamics of the 3 DoF robot
% th is a state vector having the joint angles and joint rates. This
% function returns the time derivative of the state vector (thd) that is then
% used by ODE45 to integrate the dynamic equations

%THIS IS THE ONE FUNCTION I CAN'T GET TO AUTOMATICALLY GENERATE B/C OF
%THE WAY SYMBOLIC VECTORS: REFACTOR IN TERMS OF TH1,TH2... ACCORDINGLY
function Xdot=ComputeMLSEOM(t,joint_state,tau)
    th1=joint_state(1);
    th2=joint_state(2);
    th3=joint_state(3);
    thd1=joint_state(4);
    thd2=joint_state(5);
    thd3=joint_state(6);
%------------------------------------------------------
    D=ComputeD(th1,th2,th3);
    C=ComputeC(th1,th2,th3,thd1,thd2,thd3);
    %g=[0; -(m2*9.81*r1+m3*9.81*l1)*cos(th2)-m3*r2*g*cos(th2+th3);-m3*9.81*r2*cos(th2+th3)];
    g=ComputeG(th1,th2,th3);
%------------------------------------------------------
    Xdot=[[thd1;thd2;thd3];inv(D)*(tau-C*[thd1;thd2;thd3]-g)];
%--------------------------------------------------=---
end