% This function gives the dynamics of the robot given the various matrices
% D, C, g
% th is a state vector having the joint angles and joint rates. This
% function returns the time derivative of the state vector (thd) that is then
% used by ODE45 to integrate the dynamic equations

function Xdot=ComputeEOM(t,joint_state,tau)
DOF=size(joint_state,1)/2;
    if (DOF==2)
        th1=joint_state(1);
        th2=joint_state(2);
        thd1=joint_state(3);
        thd2=joint_state(4);
    %------------------------------------------------------
        D=ComputeD(th1,th2);
        C=ComputeC(th1,th2,thd1,thd2);
        g=ComputeG(th1,th2);
        Xdot=[[thd1;thd2];inv(D)*(tau-C*[thd1;thd2]-g)];
    elseif (DOF==3)
        th1=joint_state(1);
        th2=joint_state(2);
        th3=joint_state(3);
        thd1=joint_state(4);
        thd2=joint_state(5);
        thd3=joint_state(6);
    %------------------------------------------------------
        D=ComputeD(th1,th2,th3);
        C=ComputeC(th1,th2,th3,thd1,thd2,thd3);
        g=ComputeG(th1,th2,th3);
    %------------------------------------------------------
        Xdot=[[thd1;thd2;thd3];inv(D)*(tau-C*[thd1;thd2;thd3]-g)];
    elseif (DOF==4)
        th1=joint_state(1);
        th2=joint_state(2);
        th3=joint_state(3);
        th4=joint_state(4);
        thd1=joint_state(5);
        thd2=joint_state(6);
        thd3=joint_state(7);
        thd4=joint_state(8);
    %------------------------------------------------------
        D=ComputeD(th1,th2,th3,th4);
        C=ComputeC(th1,th2,th3,th4,thd1,thd2,thd3,thd4);
        g=ComputeG(th1,th2,th3,th4);
    %------------------------------------------------------
        Xdot=[[thd1;thd2;thd3;thd4];inv(D)*(tau-C*[thd1;thd2;thd3;thd4]-g)];
    end
end