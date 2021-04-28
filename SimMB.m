%simulates the manufacturing base robot, given some parameters for MB


addpath(genpath('GEN')); %Add path for generated functions

%Derives and simplifies symbolic expressions to be used for simulation
derive=1;
J=DeriveBodyJacobians(DOF,q,w,g,derive);
D=DeriveD(J, I,m, DOF,derive);
C=DeriveC(D,DOF,derive);
gth=DeriveFK(DOF,g,w,q,derive);
N=DeriveN(DOF,gth,gravity,fc,fv,m,derive);
eom2=DeriveEOM2(D,C,N,DOF,derive);
tau=[];
for i=1:(size(t,2))
    tau=[tau,double(ComputeEOM2(js(1:DOF,i),js(DOF+1:DOF*2,i),js(2*DOF+1:3*DOF,i)))];
end
noise_js=noise_sigma.*randn(8,size(js,2))+js(1:8,:); %add gaussian noise




