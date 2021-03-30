%simulates the two link planar robot from part 4 of the notes manipulator
%see SimMLS.m for more detail on the executables
clear all;
close all;

addpath(genpath('GEN'));
%Define some physical properties of the robot
DOF=2;
I(:,:,1)=3*eye(3);
I(:,:,2)=2*eye(3);
m=[1;1];
g(:,:,1,1)=[1 0 0 2; 0 1 0 0;0 0 1 0;0 0 0 1];%the link end home frames
g(:,:,2,1)=[1 0 0 4; 0 1 0 0;0 0 1 0;0 0 0 1];
g(:,:,1,2)=[1 0 0 1; 0 1 0 0;0 0 1 0;0 0 0 1]; %the COM home frames
g(:,:,2,2)=[1 0 0 3; 0 1 0 0;0 0 1 0;0 0 0 1];
w=[0 0 1;0 0 1]';
q=[0 0 0;2 0 0]';
gravity=[0;-9.81;0];
fc=[0;0];
fv=[6;10];
derive=true;

J=DeriveBodyJacobians(DOF,q,w,g,derive);
D=DeriveD(J, I,m, DOF,derive);
C=DeriveC(D,DOF,derive);
gth=DeriveFK(DOF,g,w,q,derive);
N=DeriveN(DOF,gth,gravity,fc,fv,derive);
eqom=DeriveEOM(D,C,N,DOF,derive);

Tf=10; %end of sim (start of sim should always be t=0)
dt=0.01; %timestep
t=0:dt:Tf; %timesteps for changing torque
dt=0.01; %time step for recording joint state
js=[]; %joint state (position and velocity)
js(:,1)=zeros(DOF*2,1);
tau=10*[sin(0:0.01:10);cos(0:0.01:10)];

func_h=@(t,js)ComputeEOM(t,js,tau(:,1+floor(t/dt)));
ODEoptions=odeset('RelTol',1e-6);
for i=1:(size(t,2)-1)
    [t_temp,js_temp]=ode45(func_h,[t(i),t(i+1)],js(:,end),ODEoptions);

    js=[js js_temp(end,:)']; %only last element is recorded
    %plug in js into eqom and compare with next desired js to find next tau
end


 
