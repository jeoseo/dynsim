% This function gives the dynamics of the two link planar
% th is a state vector having the joint angles and joint rates. This
% function returns the time derivative of the state vector (thd) that is then
% used by ODE45 to integrate the dynamic equations

%THIS IS THE ONE FUNCTION I CAN'T GET TO AUTOMATICALLY GENERATE B/C OF
%THE WAY SYMBOLIC VECTORS: REFACTOR IN TERMS OF TH1,TH2... ACCORDINGLY
function Xdot=ComputeTLPEOM(t,joint_state,tau)
    th1=joint_state(1);
    th2=joint_state(2);
    thd1=joint_state(3);
    thd2=joint_state(4);
%------------------------------------------------------
    D=ComputeD(th1,th2);
    C=ComputeC(th1,th2,thd1,thd2);
    g=ComputeG(th1,th2);
%------------------------------------------------------
    Xdot=[[thd1;thd2];inv(D)*(tau-C*[thd1;thd2]-g)];
%--------------------------------------------------=---
end