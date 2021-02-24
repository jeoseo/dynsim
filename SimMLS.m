%simulates the MLS 4.3 manipulator

clear all; %Clear workspace and figures
close all;
addpath(genpath('GEN')); %Add path for generated functions

%Define some physical properties of the robot
DOF=3; %degrees of freedom
I(:,:,1)=eye(3);%inertia tensors of each link, measured from the COM
I(:,:,2)=eye(3);
I(:,:,3)=eye(3);
m=[1;1;1]; %mass of each link
r=[1;1;1];%not required, used here to define g
l=[2;2;2];%not required, used here to define g
tau=[0;0;0]; %joint torques, does not have to be constant and can instead
             %defined in the dynamic simulation loop below
%g is a 4x4xDOFxk matrix. Each 4x4 is a homogenous transform of a frame in
%the home configuration
%The 3rd dimension defines which link the frame corresponds to
%The 4th dimenstion defines which frame of the link it corresponds to: if
%k=1 it is for the link's tip frame, k=2 for the COM, k>=3 for any
%auxiliary frames
g(:,:,1,1)=[1 0 0 0; 0 1 0 0;0 0 1 l(1);0 0 0 1];%the link end home frames
g(:,:,2,1)=[1 0 0 0; 0 1 0 l(2);0 0 1 l(1);0 0 0 1];
g(:,:,3,1)=[1 0 0 0; 0 1 0 l(2)+l(3);0 0 1 l(1);0 0 0 1];
g(:,:,1,2)=[1 0 0 0; 0 1 0 0;0 0 1 r(1);0 0 0 1]; %the COM home frames
g(:,:,2,2)=[1 0 0 0; 0 1 0 r(2);0 0 1 l(1);0 0 0 1];
g(:,:,3,2)=[1 0 0 0; 0 1 0 l(2)+r(3);0 0 1 l(1);0 0 0 1];
w=[0 0 1;-1 0 0;-1 0 0]'; %Joint twist direction in the base frame
q=[0 0 0;0 0 l(1);0 l(2) l(1)]'; %Location of the twist in the base frame
gravity=[0;0;-9.81];%direction and magnitude of gravity

%Derives and simplifies symbolic expressions to be used for simulation
J=DeriveBodyJacobians(DOF,q,w,g);
D=DeriveD(J, I,m, DOF);
C=DeriveC(D,DOF);
gth=DeriveFK(DOF,g,w,q);
G=DeriveG(DOF,gth,gravity);

T0=0; %start of sim
Tf=10; %end of sim
dT=0.1; %timestep for changing torque (tau)
T=T0:dT:Tf; %Vector storing these broader timesteps
t=0; %stores all actual time steps run
js=[]; %joint state (position and velocity), correspondent to t

js(:,1)=zeros(DOF*2,1); %joint state is defined as [position;velocity]
for i=1:(size(T,2)-1)
    %Solve dynamics
    ODEoptions=odeset('RelTol',1e-6,'InitialStep',0.01);
    [t_temp,js_temp]=ode45(@ComputeEOM,[T(i) T(i+1)],js(:,size(js,2)),ODEoptions,tau);
    
    %Store results
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
    %IMPORTANT: this line must change with DOF
    gth=ComputeFK(js(1,i),js(2,i),js(3,i)); 
    
    DrawRobot(gth,0.2);
    title(t(i));
    drawnow;
end
 
