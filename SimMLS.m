%simulates the MLS 4.3 manipulator
clear all;
close all;


%Define some physical properties of the robot
DOF=3;
I(:,:,1)=eye(3);
I(:,:,2)=eye(3);
I(:,:,3)=eye(3);
m=[1;1;1];
r=[1;1;1];
l=[2;2;2];
tau=[0;0;0];
g(:,:,1,1)=[1 0 0 0; 0 1 0 0;0 0 1 l(1);0 0 0 1];%the link end home frames
g(:,:,2,1)=[1 0 0 0; 0 1 0 l(2);0 0 1 l(1);0 0 0 1];
g(:,:,3,1)=[1 0 0 0; 0 1 0 l(2)+l(3);0 0 1 l(1);0 0 0 1];
g(:,:,1,2)=[1 0 0 0; 0 1 0 0;0 0 1 r(1);0 0 0 1]; %the COM home frames
g(:,:,2,2)=[1 0 0 0; 0 1 0 r(2);0 0 1 l(1);0 0 0 1];
g(:,:,3,2)=[1 0 0 0; 0 1 0 l(2)+r(3);0 0 1 l(1);0 0 0 1];
w=[0 0 1;-1 0 0;-1 0 0]';
q=[0 0 0;0 0 l(1);0 l(2) l(1)]';
gravity=[0;0;-9.81];

J=DeriveBodyJacobians(DOF,q,w,g);
D=DeriveD(J, I,m, DOF);
C=DeriveC(D,DOF);
gth=DeriveFK(DOF,g,w,q);
G=DeriveG(DOF,gth,gravity);

T0=0; %start of sim
Tf=10; %end of sim
dT=0.1; %timestep for changing torque (tau)
T=T0:dT:Tf; %time 
t=0; %stores all actual time steps run
js=[]; %joint state (position and velocity)

js(:,1)=zeros(DOF*2,1);
for i=1:(size(T,2)-1)
    ODEoptions=odeset('RelTol',1e-6,'InitialStep',0.01);
    [t_temp,js_temp]=ode45(@ComputeMLSEOM,[T(i) T(i+1)],js(:,size(js,2)),ODEoptions,tau);
    
    t=[t; t_temp];
    js=[js js_temp'];

end
%Setup the figure and start drawing
figure();
set(gcf,'color','w');
set(gcf,'position',[0 0 1400 900]);
xlim([-5 5]);
ylim([-5 5]);
zlim([-5 5]);
view(30,30);
for i=1:20:size(js,2) %specify framerate
    gth=ComputeFK(js(1,i),js(2,i),js(3,i));
    DrawRobot(gth,0.2);
    title(t(i));
    drawnow;
end
 
