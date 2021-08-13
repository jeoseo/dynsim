%Define unknown physical properties of the robot
%in this, case we basically know nothing about the inertias or friction
DOF=4; %degrees of freedom
It=sym('It',[6,DOF]);
assume(It,'real');
I=sym(I);
I(:,:,1)=[It(1,1),It(2,1),It(3,1);
          It(2,1),It(4,1),It(5,1);
          It(3,1),It(5,1),It(6,1)];
I(:,:,2)=[It(1,2),It(2,2),It(3,2);It(2,2),It(4,2),It(5,2);It(3,2),It(5,2),It(6,2)];
I(:,:,3)=[It(1,3),It(2,3),It(3,3);It(2,3),It(4,3),It(5,3);It(3,3),It(5,3),It(6,3)];
I(:,:,4)=[It(1,4),It(2,4),It(3,4);It(2,4),It(4,4),It(5,4);It(3,4),It(5,4),It(6,4)];
m=sym('m',[DOF,1]);
assume(m,'real');
fv=sym('fv',[DOF,1]); %viscous friction unknown
assume(fv,'real');
fc=sym('fc',[DOF,1]);
assume(fc,'real');

r=sym('r',[3,DOF]);

assume(r,'real');
g=sym(g);
g(:,:,1,2)=eye(4); %COM
g(1:3,4,1,2)=g(1:3,4,1,2)+r(:,1);
g(:,:,2,2)=g(:,:,1,1);
g(1:3,4,2,2)=g(1:3,4,2,2)+r(:,2);
g(:,:,3,2)=g(:,:,2,1);
g(1:3,4,3,2)=g(1:3,4,3,2)+r(:,3);
g(:,:,4,2)=g(:,:,3,1);
g(1:3,4,4,2)=g(1:3,4,4,2)+r(:,4);


