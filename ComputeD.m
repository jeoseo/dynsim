function D = ComputeD(th1,th2)
%COMPUTED
%    D = COMPUTED(TH1,TH2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    23-Feb-2021 00:36:32

t2 = cos(th2);
t3 = t2.*2.0;
t4 = t3+2.0;
D = reshape([t2.*4.0+8.0,t4,t4,2.0],[2,2]);