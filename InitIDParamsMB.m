%This takes the hardcoded results of one identification run so we can try
%computed torque control with it
addpath(genpath('GEN'));
%Define some physical properties of the robot
DOF=4;
I(:,:,1)=4*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,2)=2*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,3)=2*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,4)=1*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
m=[8;4;4;2];
%r represents location of COM, in distance from the end of the link
r=[0.01, 0.01,0.01,0.01;
   -0.1 -0.2 -0.15 -0.05;
   -0.1 0.02 -0.02 0.02];
g(:,:,1,1)=[1 0 0 0; 0 1 0 .185;0 0 1 .197;0 0 0 1];%the link end home config frames
g(:,:,2,1)=[1 0 0 0; 0 1 0 .535;0 0 1 .197;0 0 0 1];
g(:,:,3,1)=[1 0 0 0; 0 1 0 .885;0 0 1 .197;0 0 0 1];
g(:,:,4,1)=[1 0 0 0; 0 1 0 1.025;0 0 1 .197;0 0 0 1];
g(:,:,1,2)=g(:,:,1,1); %center of mass locations
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
fc=[1,2,3,1]; %coulomb friction
fv=[1,2,3,1]; %viscous friction
Tf=10; %end of sim (start of sim should always be t=0)
dt=.01; %time step for recording joint state
t=0:dt:Tf; %timesteps for changing torque
x=[0.0167   -1.6005   -0.0199    0.2443    0.7726   -0.4712   -0.0149    0.3013   -0.7854   -0.9047   -0.0000   -0.1334   -0.0000   -1.6636   -0.0001   -0.0928];
%Optimized for faster trajectory
%x=[0.1996   -0.1078    0.2424    0.2236    0.4957   -0.3618   -0.0758    0.1708   -0.3150   -0.1387    0.1392    0.1920    0.4760   -0.2679   -0.1644    0.3170];
%x= [0.1192   -0.2487    0.2500    0.2500    0.5000    0.1619    0.4013   -0.1737   -0.5000    0.0002    0.0003   -0.4998   -0.4897    0.5000    0.1883   -0.5000];

[t,js]=ComputeJs(t,x);
%js=randn(size(js,1),size(js,2)); %for random trajectory
pos_sigma=0.1; %std deviation for white noise on position data
tau_sigma=1; %std devation for white noise on torque data