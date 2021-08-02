%We need the Jacobians to the end of each link: this is best
%done using POE. 

%COM is defined using xyz translation from the frame that the link is
%attached to
%this will return a 6 by DOF by DOF matrix, where each layer is a 6x3 Jacobian
%for each link's origin frame
function J=DeriveBodyJacobians(DOF,q,w,g,derive)
    th= sym('th',[DOF,1]); %the joint values
    assume(th,'real');
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
            if k>1 %multiple by the identity if k==1
                ad=ad*g(:,:,k-1,1);
            end
            J(:,j,k)=ComputeInvAdjoint(ad)*twists(:,j);
            c=c+1;
        end
    end
    J=simplify(expand(J));
    if (derive == true) 
        matlabFunction(J,'file','GEN/ComputeBodyJacobians.m','vars',{th});
    end
end







