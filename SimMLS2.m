%simulates the MLS 4.3 manipulator
%t is a vector of timestamps that correspond to each joint state
%js is a vector of joint states; each column corresponds to a vector of
%joint position, then velocity
%is commanded by joint state, not torque
%
%clear all; %Clear workspace and figures
close all;
addpath(genpath('GEN')); %Add path for generated functions

%----------------------------------------------------------------------
%Define some physical properties of the robot
DOF=3; %degrees of freedom
I(:,:,1)=[1 2 3; 2 4 5; 3 5 8];%inertia tensors of each link, measured from the COM
I(:,:,2)=[1 0 0;0 3 0;0 0 5]; %reminder to self to ask about equivalent inertia tensors (4,6, 6 work)
I(:,:,3)=[1 0 0;0 3 0; 0 0 5];
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
fc=[1,2,3]; %coulomb friction
fv=[1,2,3]; %viscous friction
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
t=0:dt:Tf; %timesteps for changing torque
dt=0.01; %time step for recording joint state

%hard coded joint trajectory
js=[sin(t)+.5*sin(2*t)-.5*sin(3*t);sin(t+1)+.5*sin(2*(t+1))-.5*sin(3*(t+1));sin(t-1)+.5*sin(2*(t-1))-.5*sin(3*(t-1))];
js=[js(:,2:end-1);(js(:,3:end)-js(:,1:end-2))/(2*dt)];
js=[js(:,2:end-1);(js(4:6,3:end)-js(4:6,1:end-2))/(2*dt)];
t=t(3:end-2);
noise_sigma=[0.001;0.001;0.001;0.001;0.001;0.001];


%Alternatively, we can hardcode the jointstates (probably position and
%then just take derivatives for vel and acc), eqom is tau=D*thdd+C*thd+N 
tau=[];
th=sym('th',[DOF,1]);
thd=sym('thd',[DOF,1]);
thdd=sym('thdd',[DOF,1]);
assume(th,'real');
assume(thd,'real');
assume(thdd,'real');
for i=1:(size(t,2)-1)
    tau_sub=D*js(7:9,i)+C*js(4:6,i)+N;
    for j=1:DOF %substitute values for the joint states into tau
        tau_sub=subs(tau_sub,th(j),js(j,i));
        tau_sub=subs(tau_sub,thd(j),js(DOF+j,i));
        tau_sub=subs(tau_sub,thdd(j),js(2*DOF+j,i));        
    end
    tau=[tau,double(tau_sub)];
end



%js=noise_sigma.*randn(6,size(js,2))+js(1:6,:); %add gaussian noise


