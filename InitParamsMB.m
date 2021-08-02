 addpath(genpath('GEN'));
 rng(1);
%Define some physical properties of the robot
DOF=4;
I(:,:,1)=20*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,2)=10*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,3)=10*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
I(:,:,4)=1*[1 -0.1 -0.2; -0.1 2 -0.3;-0.2 -0.3 3];
m=[20;9.5;7.3;1];
%r represents location of COM, in distance from the end of the link
r=[0.01, 0.01,0.01,0.01;
   0.1 .2 .25 .02;
   0.1 -0.02 -0.01 -0.01];
g(:,:,1,1)=[1 0 0 0; 0 1 0 .185;0 0 1 .197;0 0 0 1];%the link end home config frames
g(:,:,2,1)=[1 0 0 0; 0 1 0 .535;0 0 1 .197;0 0 0 1];
g(:,:,3,1)=[1 0 0 0; 0 1 0 .885;0 0 1 .197;0 0 0 1];
g(:,:,4,1)=[1 0 0 0; 0 1 0 1.025;0 0 1 .197;0 0 0 1];
g(:,:,1,2)=eye(4); %center of mass locations
g(1:3,4,1,2)=g(1:3,4,1,2)+r(:,1);
g(:,:,2,2)=g(:,:,1,1);
g(1:3,4,2,2)=g(1:3,4,2,2)+r(:,2);
g(:,:,3,2)=g(:,:,2,1);
g(1:3,4,3,2)=g(1:3,4,3,2)+r(:,3);
g(:,:,4,2)=g(:,:,3,1);
g(1:3,4,4,2)=g(1:3,4,4,2)+r(:,4);
w=[0 0 1;1 0 0;1 0 0 ;1 0 0]';
q=[0 0 0;0 .185 .197;0 .535 .197;0 .885 .197]';
gravity=[0;0;-9.81];
fc=[1,2,3,1]; %coulomb friction
fv=[1,2,3,1]; %viscous friction
Tf=10; %end of sim (start of sim should always be t=0)
dt=.01; %time step for recording joint state
t=0:dt:Tf; %timesteps for changing torque
%Updated values for prametrization
x=[ 0.2899   -0.4105   -0.0183    0.0195   -0.0084    0.0103    0.5635   -0.2483   -0.1638    0.0293   -0.1453    0.0166   -0.3672 -0.2073   -0.0293   -0.0859    0.2535    0.1632    0.3378   -0.3946    0.0427    0.0766   -0.1676   -0.0045];
%x=[-0.3648   -0.2254   -0.3879    0.5091   -0.1360    0.4864    0.6000   -0.2443   -0.1809   -0.0976   -0.1563    0.1055   -0.5000  -0.0710   -0.2473    0.0390   -0.3405   -0.0580   -0.1898   -0.3786    0.3944    0.1820   -0.1479   -0.1242];




[t,js]=ComputeJs(t,x);
%js=randn(size(js,1),size(js,2)); %for random trajectory
pos_sigma=0.01; %std deviation for white noise on position data
tau_sigma=1; %std devation for white noise on torque data