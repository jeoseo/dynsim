%Computes kinematics for the robot given g_st(0),joint values (th), and the joint twists (w,q) using POE

%Assumes that th,g0,w,q agree on the number of links being computed (DOF)
%g0 can have a 4th dimension k, which denotes the different frames to be
%computed for the same link (for example, the COM and the end of the link
%are two poses corresponding to the same link).
%Outputs 4x4xDOFxk symbolic matrix of homogenous transforms
%
function gth=DeriveFK(DOF,g0,w,q, derive, varargin)
    inputP=inputParser;
    addOptional(inputP,'jointType',zeros(DOF,1));
    %jointType tells us if the joint is revolute or continuum (0 or 1)
    %ex: [0,0,0,0,1,1] for the manufacturing robot
    parse(inputP,{DOF,g0,w,q, derive},varargin{:});
    th=sym('th',[DOF,1]);
    twists=ComputeJointTwist(w,q);
    gth=zeros(4,4,DOF,size(g0,4));
    gth=sym(gth);
    e=eye(4,4);%identity homegenous trasnform;
    for i=1:DOF
        e=e*ComputeExpn(twists(:,i),th(i)); %We compute this once
        for j=1:size(g0,4)
            gth(:,:,i,j)=e*g0(:,:,i,j);
        end
    end
    gth=simplify(gth);
    
    if (derive == true) 
        matlabFunction(gth,'file','GEN/ComputeFK.m','vars',{th});
    end
    
end