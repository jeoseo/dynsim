function J = ComputeBodyJacobians(th1,th2)
%COMPUTEBODYJACOBIANS
%    J = COMPUTEBODYJACOBIANS(TH1,TH2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    23-Feb-2021 00:36:31

J = reshape([0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,sin(th2).*2.0,cos(th2).*2.0+1.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0,0.0,0.0,1.0],[6,2,2]);
