%Computes the 6x6 inverse adjoint transform for some 4x4 homongenous transform


function adj=ComputeInvAdjoint(T)
    R=T(1:3,1:3);
    p=T(1:3,4);
    adj=[R' -R'*ComputeHat3(p);zeros(3) R'];
end
