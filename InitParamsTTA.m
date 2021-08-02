addpath(genpath('GEN'));
%Define some physical properties of the robot
DOF=1;
I(:,:,1)=.001*[0 0 0;0 0 0;0 0 2.2039315e02]; %Nothing but Izz matters
 m=3.76;
%r represents location of COM, in distance from the end of the link
r=.001*[2.2901048e02 ; 6.7160469e-01 ; 2.1029405e01 ];
g(:,:,1,1)=[0 0 -1 0.23; 0 1 0 0;1 0 0 0.02; 0 0 0 1];%the link end home config frames
g(:,:,1,2)=eye(4);
g(1:3,4,1,2)=g(1:3,4,1,2)+r(:,1);
w=[0 0 1]';
q=[0 0 0]';
gravity=[0;-9.81;0];
fc=0; %coulomb friction
fv=0; %viscous friction
Tf=10; %end of sim (start of sim should always be t=0)
dt=.01; %time step for recording joint state
%t=0:dt:Tf; %timesteps for changing torque
%x=[-1,1,0,0,0,0];
%[t,js]=ComputeJs(t,x);
%pos_sigma=0.00; %std deviation for white noise on position data
%tau_sigma=1; %std devation for white noise on torque data