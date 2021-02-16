%The Coriolis matrix
%J is the symbolic variable for the Jacobians for the COM's of all links
%DOF can be found from J but exists separately for readability
function C=DeriveC(D,DOF)
    C=zeros(DOF,DOF);
    C=sym(C);
    th=sym('th',[DOF,1]);
    thd=sym('thd',[DOF,1]);
    assume(th,'real');
    assume(thd,'real');
    for i=1:DOF
        for j=1:DOF
            for k=1:DOF
                C(i,j)=C(i,j)+0.5*(diff(D(i,j),th(k))+diff(D(i,k),th(j))-diff(D(j,k),th(i)))*thd(k);
            end
        end
    end
    C=simplify(C);
end