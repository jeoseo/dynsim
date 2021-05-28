% 3rd form of the equations of motion, used for linearization in 
%computed torque
% computations
function tau=DeriveEOM3(D,C,N,DOF,derive)
     syms t
     aq=sym('aq',[DOF 1]);
     thd=sym('thd',[DOF 1]);
     th=sym('th',[DOF 1]);
     tau=C*thd+N+D*aq;
     joint_state=[th;thd];
     if (derive == true) 
         matlabFunction(tau,'file','GEN/ComputeEOM3.m','vars',[{aq},{joint_state}]);
     end
end