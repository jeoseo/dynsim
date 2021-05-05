 addpath(genpath('GEN'));
%Define some physical properties of the robot
DOF=4;
I(:,:,1)=4*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,2)=2*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,3)=2*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,4)=1*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
m=[8;4;4;2];
%r represents location of COM, in distance from the start of the link
r=[0.01, 0.01,0.01,0.01;
   -0.1 -0.2 -0.15 -0.05;
   -0.1 0.02 -0.02 0.02];
g(:,:,1,1)=[1 0 0 0; 0 1 0 .185;0 0 1 .197;0 0 0 1];%the link end home frames
g(:,:,2,1)=[1 0 0 0; 0 1 0 .535;0 0 1 .197;0 0 0 1];
g(:,:,3,1)=[1 0 0 0; 0 1 0 .885;0 0 1 .197;0 0 0 1];
g(:,:,4,1)=[1 0 0 0; 0 1 0 1.025;0 0 1 .197;0 0 0 1];
g(:,:,1,2)=g(:,:,1,1);
g(1:3,4,1,2)=g(1:3,4,1,2)+r(:,1);
g(:,:,2,2)=g(:,:,2,1);
g(1:3,4,2,2)=g(1:3,4,2,2)+r(:,2);
g(:,:,3,2)=g(:,:,3,1);
g(1:3,4,3,2)=g(1:3,4,3,2)+r(:,3);
g(:,:,4,2)=g(:,:,4,1);
g(1:3,4,4,2)=g(1:3,4,4,2)+r(:,4);
w=[0 0 1;1 0 0;1 0 0 ;1 0 0]';
q=[0 0 0;0 .185 .197;0 .535 .197;0 .885 .197]';
gravity=[0;0;-9.81];
fc=[1,2,3,4]; %coulomb friction
fv=[1,2,3,4]; %viscous friction
Tf=10; %end of sim (start of sim should always be t=0)
dt=.01; %time step for recording joint state
t=0:dt:Tf; %timesteps for changing torque
%hard coded joint trajectory
   x=[ 0.2472   -0.2279    0.0221   -0.2500    0.5000   -0.1847    0.3476    0.2043   -0.2643   -0.0107    0.1946   -0.0205   -0.1544   -0.0211    0.3634    0.2321];

[t,js]=ComputeJs(t,x);
pos_sigma=0.1; %std deviation for white noise on position data
tau_sigma=1; %std devation for white noise on torque data