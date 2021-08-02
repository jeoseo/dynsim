%Assume trajectory data has already been collected or simulated, in [t,noise_js]
%Assume initial parameter estimates have also been made
th=sym('th',[DOF,1]);
thd=sym('thd',[DOF,1]);
thdd=sym('thdd',[DOF,1]);
assume(th,'real');
assume(thd,'real');
assume(thdd,'real');
derive=0;
J=DeriveBodyJacobians(DOF,q,w,g,derive);
D=DeriveD(J, I,m, DOF,derive);
C=DeriveC(D,DOF,derive);
gth=DeriveFK(DOF,g,w,q,derive);
N=DeriveN(DOF,gth,gravity,fc,fv,m,derive); 
eom2=DeriveEOM2(D,C,N,DOF,derive);

coeffCell=cell(1,DOF);
for i=1:DOF
    %here we are estimating the inertia tensor, friction, mass, and COM
    %location
    %COM location looks strange b/c r has some assumptions
    [coeff,term]=coeffs(eom2(i),{It(:);fv(:);fc(:);r(:);m(:)});
    term=arrayfun(@char, term,'uniform',0); %turn from symbolic to cell
    coeff=arrayfun(@char,coeff,'uniform',0);
    coeffCell{i}=containers.Map(term,coeff);
end
% creating the vector with the unknown inertial parameters that matter
allTerms=containers.Map('KeyType','char', 'ValueType','any');
for i=1:DOF
    keyList=keys(coeffCell{i});
    for j=1:size(keyList,2)
        if ~isKey(allTerms,keyList{j})
            allTerms(keyList{j})=0; %val in this map is meaningless
        end
    end
end
allKeyTerms=keys(allTerms);
%Create phi, the matrix that converts inertial parameters to joint torques
phi=zeros(DOF,size(allKeyTerms,2));
phi=sym(phi);

for i=1:DOF
    for j=1:size(keys(allTerms),2)
        if isKey(coeffCell{i},allKeyTerms(j))
            phi(i,j)=str2sym(convertCharsToStrings(coeffCell{i}(allKeyTerms{j})));
        end
    end
end
matlabFunction(phi,'file','GEN/ComputePhiTTA.m','vars',[{th},{thd},{thdd}]);
F=[]; %phi stacked vertically such that we can perform a least squares
tau_ext=[];%tau with the elements matching F
[t_out,filt_js]=ComputeFilteredJs(t,js,DOF,pos_sigma);
%t_out=t;
%filt_js=js;
offset=(size(t,2)-size(t_out,2))/2; %by filtering, we often have to truncate the signal
for i=1:size(filt_js,2)
    phiSub=double(ComputePhiMB(filt_js(1:DOF,i),filt_js(1+DOF:2*DOF,i),filt_js(1+2*DOF:3*DOF,i)));
    %the tau at the timestamps we are using are collected, with noise added
    tau_ext=[tau_ext;tau(:,i+offset)+randn(DOF,1).*tau_sigma];
    F=[F;phiSub]; %We stack our phi's together to do least mean squares
end
%found by row reducing F
[~,indcolF]=rref(F);
Fshrunk=F(:,indcolF);
parameter_val_shrunk=pinv(Fshrunk)*tau_ext;
parameter_val=zeros(size(F,2),1);
for i=1:size(Fshrunk,2)
    parameter_val(indcolF(i))=parameter_val_shrunk(i); %repadding zeros
end
%imp=inertial parameter map
imp=containers.Map(allKeyTerms,parameter_val);








