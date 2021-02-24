%We need the Jacobians to the center of mass of each link: this is best
%done using POE. 

%COM is defined using xyz translation from the frame that the link is
%attached to
%this will return a 6 by DOF by DOF matrix, where each layer is a 6x3 Jacobian
%for each link's COM expressed in the body frame

function J=DeriveBodyJacobians(DOF,q,w,g)
    th= sym('th',[DOF,1]); %the joint values
    twists=ComputeJointTwist(w,q);
    
    J=zeros([6,DOF,DOF]);
    J=sym(J);
    c=0;
    for k=1:DOF
        for j=1:k % some columns of jacobian are left blank, namely the ones for which the joint is beyond the COM being focused one
            ad=eye(4,4); %identity transform
            for j2=j:k %for finding the requisite adjoint transform
                ad=ad*ComputeExpn(twists(:,j2),th(j2));
            end
            ad=ad*g(:,:,k,2);
            J(:,j,k)=inv(ComputeAdjoint(ad))*twists(:,j);
            c=c+1;
        end
    end
    J=simplify(J);
    matlabFunction(J,'file','GEN/ComputeBodyJacobians.m','vars',[th]);
end







