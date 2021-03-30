%Also known as M, this is the inertia/mass matrix
%J is the symbolic variable for the Jacobians for the COM's of all links
%DOF can be found from J but exists separately for readability
function D=DeriveD(J, I,m, DOF, derive)
    th=sym('th',[DOF,1]);
    assume(th,'real');
    D=zeros(DOF,DOF);
    D=sym(D);
    for i=1:DOF
        M=zeros(6,6);
        M=sym(M);
        M(1,1)=m(i);
        M(2,2)=m(i);
        M(3,3)=m(i);
        M(4:6,4:6)=I(:,:,i);
        D=D+J(:,:,i)'*M*J(:,:,i);
    end
    D=simplify(expand(D));
    if (derive == true) 
        matlabFunction(D,'file','GEN/ComputeD.m','vars',{th});
    end;
end