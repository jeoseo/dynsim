%Define unknown physical properties of the robot
%in this, case we basically know nothing about the inertias or friction
DOF=1; %degrees of freedom
It=sym('It',[6,DOF]);
assume(It,'real');
I=[];
I=sym(I);
I(:,:,1)=[It(1,1),It(2,1),It(3,1);It(2,1),It(4,1),It(5,1);It(3,1),It(5,1),It(6,1)];
m=sym('m',[DOF,1]);
assume(m,'real');
fv=sym('fv',[DOF,1]); %viscous friction unknown
assume(fv,'real');
fc=sym('fc',[DOF,1]);
assume(fc,'real');

r=sym('r',[3,DOF]);
assume(r,'real');
g=[];
g=sym(g);
%Assumes the robot starts at the origin
g(:,:,1,1)=[1 0 0 0; 0 1 0 1.025-.885;0 0 1 0;0 0 0 1];%the link end home config frames

g(:,:,1,2)=eye(4); %COM
g(1:3,4,1,2)=g(1:3,4,1,2)+r(:,1);


w=[1 0 0]';
q=[0 0 0]';
gravity=[0;-9.81;0];

a=[    4.6555    6.2920    0.5452    0.8938    2.0300    0.0149    5.6903    2.5000];
