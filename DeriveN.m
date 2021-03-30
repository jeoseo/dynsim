%Derives the G vector in the EOM given some gth (the pose of the COM's of the robot) and universal gravitational vector
function N=DeriveN(DOF,gth,gravity,fc,fv, derive)
    th=sym('th',[DOF,1]);
    thd=sym('thd',[DOF,1]);
    assume(th,'real');
    assume(thd,'real');
    N=zeros(DOF,1);
    N=sym(N);
    P=0;
    P=sym(P);
    for i=1:DOF %Find 
        P=P-dot(gth(1:3,4,i,2),gravity); %the translational vector for the COM of the ith link
    end
    for i=1:DOF
        N(i)=diff(P,th(i))+fc(i)*sign(thd(i))+fv(i)*thd(i);
    end
    N=simplify(N);
    if (derive == true)
        matlabFunction(N,'file','GEN/ComputeN.m','vars',{th,thd});
    end
end