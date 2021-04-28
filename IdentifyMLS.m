%Assume trajectory data has already been collected or simulated with SimTLP.m, 
%specifically we need [t, js] from the ODE solver, as well as 

%Define unknown physical properties of the robot
%In this case, we pretend that we know the everything but the inertia
%tensor, but we know that the inertia tensor is diagonal.
DOF=3; %degrees of freedom
It=sym('It',[6,DOF]);
assume(It,'real');
I=sym(I);
I(:,:,1)=[It(1,1),It(2,1),It(3,1);It(2,1),It(4,1),It(5,1);It(3,1),It(5,1),It(6,1)];
I(:,:,2)=[1,It(2,2),It(3,2);It(2,2),It(4,2),It(5,2);It(3,2),It(5,2),It(6,2)];
I(:,:,3)=[1,It(2,3),It(3,3);It(2,3),It(4,3),It(5,3);It(3,3),It(5,3),It(6,3)];
fv=sym('fv',[DOF,1]); %viscous friction unknown
fc=sym('fc',[DOF,1]);
m=sym('m',[DOF,1]);
r=sym('r',[DOF,1]);
assume(fc,'real');
assume(m,'real');
assume(r,'real');
g=sym(g);
g(:,:,1,1)=[1 0 0 0; 0 1 0 0;0 0 1 l(1);0 0 0 1];%the link end home frames
g(:,:,2,1)=[1 0 0 0; 0 1 0 l(2);0 0 1 l(1);0 0 0 1];
g(:,:,3,1)=[1 0 0 0; 0 1 0 l(2)+l(3);0 0 1 l(1);0 0 0 1];
g(:,:,1,2)=[1 0 0 0; 0 1 0 0;0 0 1 r(1);0 0 0 1]; %the COM home frames
g(:,:,2,2)=[1 0 0 0; 0 1 0 r(2);0 0 1 l(1);0 0 0 1];
g(:,:,3,2)=[1 0 0 0; 0 1 0 l(2)+r(3);0 0 1 l(1);0 0 0 1];


derive=false;

th=sym('th',[DOF,1]);
thd=sym('thd',[DOF,1]);
thdd=sym('thdd',[DOF,1]);
assume(th,'real');
assume(thd,'real');
assume(thdd,'real');
J=DeriveBodyJacobians(DOF,q,w,g,derive);
D=DeriveD(J, I,m, DOF,derive);
C=DeriveC(D,DOF,derive);
gth=DeriveFK(DOF,g,w,q,derive);
N=DeriveN(DOF,gth,gravity,fc,fv,derive); 
%We instead need to find a different EOM representation
eqom=D*thdd+C*thd+N;
eqom=simplify(expand(eqom));

coeffCell=cell(1,DOF);
for i=1:DOF
    [coeff,term]=coeffs(eqom(i),{It(:);fv;fc;m;r});
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
F=[]; %phi stacked vertically such that we can perform a least squares
tau_ext=[];%tau with the elements matching F
for i=3:10:size(js,2)-2 %trimmed a little so we can get thdd,also downsampled
    phiSub=phi;
    for j=1:DOF
        phiSub=subs(phiSub,th(j),js(j,i));
        phiSub=subs(phiSub,thd(j),js(DOF+j,i));
        %This sampling method is from page 11 of Khosla 1987
        phiSub=subs(phiSub,thdd(j),(js(DOF+j,i+2)*2+js(DOF+j,i+1)-js(DOF+j,i-1)-js(DOF+j,i-2)*2)/(10*dt));        
    end
    phiSub=double(phiSub);
    tau_ext=[tau_ext;tau(:,i)]; %the tau at the timestamps we are using are collected
    F=[F;phiSub]; %We stack our phi's together to do least mean squares
end
%We perform least mean squares, but we exclude the first column of F as it
%should correspond to 1, the constant term every time.
%parameter_val=lsqr(F,tau_ext);
parameter_val=pinv(F(:,2:end))*(tau_ext-F(:,1));
parameter_val=[1;parameter_val]; %add the 1 back in to match with the keyTerms
inertial_parameter_map=containers.Map(allKeyTerms,parameter_val);







