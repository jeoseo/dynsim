 addpath(genpath('GEN'));
%Define some physical properties of the robot
DOF=4;
I(:,:,1)=1*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,2)=2*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,3)=3*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,4)=4*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
m=[10;8;6;4];
%r represents location of COM, in distance from the start of the link
r=-1*[0.1 0.1 -0.1 -0.1;
   0.1 0.2 0.3 0.4;
   0.1 0 0 0];
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
   x=[ 0.0018   -0.1720   -0.0520    0.1389    0.8381    0.2550   -0.3441   -0.2084   -0.8077    0.1776   -0.6924   -0.2189    0.0565   -1.0000   -0.7034    0.1550];

 js=[x(1)+x(2)*sin(.2*pi*t)+x(3)*sin(.4*pi*t)+x(4)*sin(.6*pi*t);
     x(5)+x(6)*sin(.2*pi*t)+x(7)*sin(.4*pi*t)+x(8)*sin(.6*pi*t);
     x(9)+x(10)*sin(.2*pi*t)+x(11)*sin(.4*pi*t)+x(12)*sin(.6*pi*t);
     x(13)+x(14)*sin(.2*pi*t)+x(15)*sin(.4*pi*t)+x(16)*sin(.6*pi*t)];
 
 js=[js(:,2:end-1);(js(:,3:end)-js(:,1:end-2))/(2*dt)];
 js=[js(:,2:end-1);(js(5:8,3:end)-js(5:8,1:end-2))/(2*dt)]; 
t=t(3:end-2);
%js=2*rand(12,size(t,2))-1; %clearly impossible, but should be "exciting"
noise_sigma=0.001*ones(8,1);