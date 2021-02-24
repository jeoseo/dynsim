%Derives the G vector in the EOM given some gth (the pose of the COM's of the robot) and universal gravitational vector
function G=DeriveG(DOF,gth,gravity)
    th=sym('th',[DOF,1]);
    G=zeros(DOF,1);
    G=sym(G);
    P=0;
    P=sym(P);
    for i=1:DOF
        P=P-dot(gth(1:3,4,i,2),gravity); %the translational vector for the COM of the ith link
    end
    for i=1:DOF
        G(i)=diff(P,th(i));
    end
    G=simplify(G);
    matlabFunction(G,'file','GEN/ComputeG.m','vars',[th]);
end