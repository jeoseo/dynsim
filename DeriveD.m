%Also known as M, this is the inertia/mass matrix
%J is the symbolic variable for the Jacobians for the COM's of all links
%DOF can be found from J but exists separately for readability
function D=DeriveD(J, DOF)

    l=sym('l',[DOF,1]);
    r=sym('r',[DOF,1]);
    th=sym('th',[DOF,1]);
    assume(l,'real');
    assume(r,'real');
    assume(th,'real'); %everything is real

    
    D=zeros(DOF,DOF);
    D=sym(D);
    m=sym('m',[DOF,1]);
    %I=sym('I',[3,3,DOF]); %for now, just a simple 3x3 matrix for this
    Ixx=sym('Ixx',[DOF,1]);
    Iyy=sym('Iyy',[DOF,1]);
    Izz=sym('Izz',[DOF,1]);

    sym M;
    
    
    for i=1:DOF
        M=zeros(6,6);
        M=sym(M);
        M(1,1)=m(i);
        M(2,2)=m(i);
        M(3,3)=m(i);
        M(4,4)=Ixx(i);
        M(5,5)=Iyy(i);
        M(6,6)=Izz(i);
        D=D+J(:,:,i)'*M*J(:,:,i);
    end
    D=simplify(D);
end