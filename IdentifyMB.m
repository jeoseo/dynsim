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
% creating the vector with the different polynomial combianations of the 
% unknown inertial parameters
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
matlabFunction(phi,'file','GEN/ComputePhiMB.m','vars',[{th},{thd},{thdd}]);


F=[]; %phi stacked vertically such that we can perform a least squares
tau_ext=[];%tau with the elements matching F
for i=3:size(noise_js,2)-2
    noise_js(2*DOF+1:3*DOF,i)=(noise_js(DOF+1:2*DOF,i+2).*2+noise_js(DOF+1:2*DOF,i+1)-noise_js(DOF+1:2*DOF,i-1)-noise_js(DOF+1:2*DOF,i-2).*2)./(10*dt);
    phiSub=double(ComputePhiMB(noise_js(1:DOF,i),noise_js(1+DOF:2*DOF,i),noise_js(1+2*DOF:3*DOF,i)));
    tau_ext=[tau_ext;tau(:,i)]; %the tau at the timestamps we are using are collected
    F=[F;phiSub]; %We stack our phi's together to do least mean squares
end
%parameter_val=lsqr(F,tau_ext);
%found by row reducing F
indcolF=[ 1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    20    21    22    23    24    25    26    27  29   30     31    32    33    34    35    36    37];
Fshrunk=F(:,indcolF);
parameter_val_shrunk=pinv(Fshrunk)*tau_ext;
parameter_val=[parameter_val_shrunk(1:16);0;0;0;parameter_val_shrunk(17:24);0;parameter_val_shrunk(25:end)]; %hardcoded

%imp=inertial parameter map
imp=containers.Map(allKeyTerms,parameter_val);








