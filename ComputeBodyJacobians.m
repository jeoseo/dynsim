%Given some symbolic form of the jacobians, compute in terms of a known
%theta

%the symbolic form is of the manipulator on page 172 of MLS
%theta

function J=ComputeBodyJacobians(theta1,theta2,theta3)
    syms  r1 r2 r3 l1 l2 l3 real
    J=zeros([6,3,3]);
    J=sym(J);
    J(:,:,1)=[0, 0, 0;0, 0, 0;0, 0, 0;0, 0, 0;0, 0, 0;1, 0, 0];
    J(:,:,2)=[-r1*cos(theta2), 0, 0; 0, 0, 0; 0, -r1, 0; 0, -1, 0; -sin(theta2), 0, 0; cos(theta2), 0, 0];
    J(:,:,3)=[- r2*cos(theta2 + theta3) - l1*cos(theta2), 0, 0; 0, l1*sin(theta3), 0; 0, - r2 - l1*cos(theta3), -r2; 0, -1, -1; -sin(theta2 + theta3), 0, 0; cos(theta2 + theta3), 0, 0];
    J=simplify(J);


end
