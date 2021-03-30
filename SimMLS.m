%simulates the MLS 4.3 manipulator
%t is a vector of timestamps that correspond to each joint state
%js is a vector of joint states; each column corresponds to a vector of
%joint position, then velocity
%
%clear all; %Clear workspace and figures
close all;
addpath(genpath('GEN')); %Add path for generated functions

%----------------------------------------------------------------------
%Define some physical properties of the robot
DOF=3; %degrees of freedom
I(:,:,1)=[1 2 3; 2 4 5; 3 5 8];%inertia tensors of each link, measured from the COM
I(:,:,2)=[1 0 0;0 3 0;0 0 5]; %reminder to self to ask about equivalent inertia tensors (4,6, 6 work)
I(:,:,3)=[1 0 0; 0 3 0; 0 0 5];
m=[1;1.3;1.6]; %mass of each link
r=[1;1.5;1];%used here to define g, is the distance to COM from each link
l=[2;2;3];%used to define g, is the length of each link
%g is a 4x4xDOFxk matrix. Each 4x4 is a homogenous transform of a frame 
%in the home configuration
%The 3rd dimension defines which link the frame corresponds to
%The 4th dimenstion defines which frame of the link it corresponds to
%if k=1 it is for the link's tip frame, k=2 for the COM, k>=3 for any
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
fc=[0;0;0]; %coulomb friction
fv=[1;2;3]; %viscous friction
derive=true; %want to generate compute functions
%----------------------------------------------------------------------

%Derives and simplifies symbolic expressions to be used for simulation
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
% simulated gaussian noise os the joint state, where velocity and position 
%are altered by a constant gaussian of std deviation below
%first three values are for position noise, next 3 are for velocity noise
noise_sigma=[0.01;0.01;0.01;0.01;0.01;0.01];
js(:,1)=zeros(DOF*2,1);%joint state is initialized

tau=10*[sin(0:0.01:10);cos(0:0.01:10);sin(0:0.01:10)]; %some sinusoids for tau
ODEoptions=odeset('RelTol',1e-6);
for i=1:(size(t,2)-1)
    [t_temp,js_temp]=ode45(@ComputeEOM,[t(i),t(i+1)],js(:,end),ODEoptions,tau(:,i));
    js=[js js_temp(end,:)']; %only last element is recorded
end

js=noise_sigma.*randn(size(js,1),size(js,2))+js; %add gaussian noise


