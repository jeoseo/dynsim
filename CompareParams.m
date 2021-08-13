%Creates a table ofidentified versus the simulated "real" parameters


%Replace this line with an initialization of the real parameters
InitSimParamsMB();
%hard coded vector of parameters
real_params=[I(1,1,2);I(1,1,3);I(1,1,4);I(2,1,2);I(2,1,3);I(2,1,4);...
    I(3,1,2);I(3,1,3);I(3,1,4);I(2,2,2);I(2,2,3);I(2,2,4);...
    I(2,3,2);I(2,3,3);I(2,3,4);I(3,3,1);I(3,3,2);I(3,3,3);I(3,3,4);...
    fc(1);fc(2);fc(3);fc(4);fv(1);fv(2);fv(3);fv(4);m(2);...
    m(2)*r(2,2);m(2)*r(3,2);m(3);m(3)*r(2,3);m(3)*r(3,3);m(4);...
    m(4)*r(2,4);m(4)*r(3,4)];

%Populates the covariance matrices in case someone wants them
C_tau=eye(size(Fshrunk,1))*tau_sigma;
C_F=(Fshrunk'*C_tau^(-1)*Fshrunk)^(-1);
%[U,S,V]=svd(C_F);
%diagS=diag(S);



%Because our parameters are dependent, rearrange the parameters in
%terms of what the identification procedure should produce
rrefF=rref(F);
real_params2=rrefF(1:rank(F),:)*real_params;
real_params2=double(real_params2);
diagC_F=diag(C_F);
std_dev=sqrt(diagC_F);
%this can then be compared to parameter_val_shrunk and the covariance
%matrix

%Find how many std_deviations above/below the real value is (bs checking)
std_dev_error=zeros(size(real_params2,1),1);
for i=1:size(real_params2,1)
    std_dev_error(i)=(parameter_val_shrunk(i)-real_params2(i))/sqrt(diagC_F(i));
end
%display
T=table(real_params2,parameter_val_shrunk,std_dev,std_dev_error)
mean_std_dev_error=mean(abs(std_dev_error));