function eom2=DeriveEOM2(D,C,N,DOF,derive)
    th=sym('th',[DOF,1]);
    thd=sym('thd',[DOF,1]);
    thdd=sym('thdd',[DOF,1]);
    assume(th,'real');
    assume(thd,'real');
    assume(thdd,'real');
    eom2=D*thdd+C*thd+N;
    eom2=simplify(expand(eom2));
    if (derive == true) 
        matlabFunction(eom2,'file','GEN/ComputeEOM2.m','vars',[{th},{thd},{thdd},]);
    end
end