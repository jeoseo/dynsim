%simulates the two link planar robot from part 4 of the notes manipulator
%see SimMLS.m for more detail on the executables
function [t,js]=SimTLP()
    clear all;
    close all;

    addpath(genpath('GEN'));
    %Define some physical properties of the robot
    DOF=2;
    I(:,:,1)=eye(3);
    I(:,:,2)=eye(3);
    m=[1;1];
    tau=[0;0];
    g(:,:,1,1)=[1 0 0 2; 0 1 0 0;0 0 1 0;0 0 0 1];%the link end home frames
    g(:,:,2,1)=[1 0 0 4; 0 1 0 0;0 0 1 0;0 0 0 1];
    g(:,:,1,2)=[1 0 0 1; 0 1 0 0;0 0 1 0;0 0 0 1]; %the COM home frames
    g(:,:,2,2)=[1 0 0 3; 0 1 0 0;0 0 1 0;0 0 0 1];
    w=[0 0 1;0 0 1]';
    q=[0 0 0;2 0 0]';
    gravity=[0;-9.81;0];

    J=DeriveBodyJacobians(DOF,q,w,g);
    D=DeriveD(J, I,m, DOF);
    C=DeriveC(D,DOF);
    gth=DeriveFK(DOF,g,w,q);
    G=DeriveG(DOF,gth,gravity);
    EOM=DeriveEOM(D,C,G,DOF);

    T0=0; %start of sim
    Tf=10; %end of sim
    dT=0.1; %timestep for changing torque (tau)
    T=T0:dT:Tf; %timesteps for changing torque
    t=0; %stores all actual time steps run
    js=[]; %joint state (position and velocity)

    js(:,1)=zeros(DOF*2,1);
    for i=1:(size(T,2)-1)
        ODEoptions=odeset('RelTol',1e-6,'InitialStep',0.01);
        [t_temp,js_temp]=ode45(@ComputeEOM,[T(i) T(i+1)],js(:,end),ODEoptions,tau);

        t=[t  t_temp'];
        js=[js js_temp'];

    end
end

 
