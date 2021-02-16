%Generates the joint twists for revolute joints in the home configuration
% this is [q x w; w], aka linear velocity then angular velocity

%For revolute joints:
%w is the direction in which the screw axis points in world frame
%q is any point on the screw axis expressed in world frame
%w=[w1 w2 ...], so 3 by n
%q=[q1 q2 ...]
%outputs 6 by n
function joint_screws=ComputeJointTwist(w,q)
    for i=1:size(q,2)    
        joint_screws(:,i)=[cross(q(:,i),w(:,i));w(:,i)];
    end
end

