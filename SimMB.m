%simulates the manufacturing base robot
%see SimMLS.m for more detail on the executables
clear all;
close all;

addpath(genpath('GEN'));
%Define some physical properties of the robot
DOF=4;
I(:,:,1)=eye(3);
I(:,:,2)=eye(3);
I(:,:,3)=eye(3);
I(:,:,4)=eye(3);
m=[1;1;1;1];
tau=[0;0;0;0];
g(:,:,1,1)=[1 0 0 0; 0 1 0 .185;0 0 1 .197;0 0 0 1];%the link end home frames
g(:,:,2,1)=[1 0 0 0; 0 1 0 .535;0 0 1 .197;0 0 0 1];
g(:,:,3,1)=[1 0 0 0; 0 1 0 .885;0 0 1 .197;0 0 0 1];
g(:,:,4,1)=[1 0 0 0; 0 1 0 1.025;0 0 1 .197;0 0 0 1];
g(:,:,1,2)=[1 0 0 0; 0 1 0 0;0 0 1 .197;0 0 0 1];%COM
g(:,:,2,2)=[1 0 0 0; 0 1 0 (.535+.185)/2;0 0 1 .197;0 0 0 1];
g(:,:,3,2)=[1 0 0 0; 0 1 0 (.885+535)/2;0 0 1 .197;0 0 0 1];
g(:,:,4,2)=[1 0 0 0; 0 1 0 (1.025+.885)/2;0 0 1 .197;0 0 0 1];
w=[0 0 1;1 0 0;1 0 0 ;1 0 0]';
q=[0 0 0;0 .185 .197;0 .535 .197;0 .885 .197]';
gravity=[0;0;-9.81];

J=DeriveBodyJacobians(DOF,q,w,g);
D=DeriveD(J, I,m, DOF);
C=DeriveC(D,DOF);
gth=DeriveFK(DOF,g,w,q);
G=DeriveG(DOF,gth,gravity);

T0=0; %start of sim
Tf=10; %end of sim
dT=0.1; %timestep for changing torque (tau)
T=T0:dT:Tf; %timesteps for changing torque
t=0; %stores all actual time steps run
js=[]; %joint state (position and velocity)

js(:,1)=zeros(DOF*2,1);
for i=1:(size(T,2)-1)
    ODEoptions=odeset('RelTol',1e-6,'InitialStep',0.01);
    [t_temp,js_temp]=ode45(@ComputeEOM,[T(i) T(i+1)],js(:,size(js,2)),ODEoptions,tau);
    
    t=[t; t_temp];
    js=[js js_temp'];

end
%Setup the figure and start drawing
figure();
set(gcf,'color','w');
set(gcf,'position',[0 0 1400 900]);
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
view(30,30);
for i=1:20:size(js,2) %specify framerate
    gth=ComputeFK(js(1,i),js(2,i),js(3,i),js(4,i));
    DrawRobot(gth,0.05);
    title(t(i));
    drawnow;
end
 
