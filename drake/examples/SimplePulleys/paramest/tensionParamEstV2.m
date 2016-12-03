function [] = tensionParamEstV2()
%% Set the following parameters to false after running it for the first time:

%% Parse in Optitrack Capture Data
%loaded preprocessed data set

dataSetName = 'set5';
name = '16-May-2015 20_46_41';
filename = ['~/soft_modeling_repo/dev/tracking/data/',dataSetName,'/',name];

appendix = '_preprocessed';
fullFilename = [filename,appendix,'.mat'];
load(fullFilename)


%% flags that define what part to execute
calcBSurfaceFlag = false;
calcOneDofProblem = true; % calculating Thetas
calcThetaFlag = false;
estimateParamsFminConFlag = false; %using fmin con to estimate the parameters for each contact phase individually
calcAllOfOneCombinedFlag = false; % using fmincon to estimate the parameters for all contact phases as one data set


% timeStepsNC,zNC,zdNC,zddNC,timeStepsLargeNC,zLargeNC,zdLargeNC,zddLargeNC,zfitNC


% generatePlot = true;

%% Estimate bsurface
if(calcBSurfaceFlag)

numOfSetsNC = size(zdNC,1);
bsurfaceEstViscous = zeros(numOfSetsNC,2);
force = cell(numOfSetsNC,1);
forcehat = cell(numOfSetsNC,1);
r = cell(numOfSetsNC,1);
bsurfaceEst = zeros(numOfSetsNC,3);

rangeTested = 1:numOfSetsNC;

figure(30);clf;hold on;
title('z')
figure(31);clf;hold on;
title('zd')
figure(32);clf;hold on;
title('zdd')
figure(40);clf;hold on;
title('force')
figure(41);clf;hold on;
title('residuals')

for j = rangeTested
t = timeStepsNC{j};
q = [zNC{j}'];
qd = [zdNC{j}'];
qdd = [zddNC{j}'];
[bsurfaceEst(j,:),force{j},forcehat{j},r{j},~] = ordinaryLeastSquaresNoContact(qd, qdd, angleDeg, mdisc);

%bsurfaceEstTustin(j,:) = ordinaryLeastSquaresNoContactTustin(qd, qdd, angleDeg, mdisc);

figure(30);
plot(t,q,'k.');
plot(t,q,'k');
figure(31);
plot(t,qd,'k.');
plot(t,qd,'k');
figure(32);
plot(t,qdd,'k.');
plot(t,qdd,'k');
figure(40)
plot(t,force{j},t,force{j},'.');
plot(t,forcehat{j},'r',t,forcehat{j},'r*');
h = legend('$f$','$f$','$\hat{f}$','$\hat{f}$');
set(h,'Interpreter','Latex');
figure(41)
plot(t,r{j},'.');
plot(t,r{j});

end

figure(30);
axis([-inf inf zTouch inf])

for i = 1:1
bSlip = bsurfaceEst(rangeTested,i);
bSlipMean = mean(bSlip);

figure(33+i); clf; hold on; 
plot(rangeTested,bSlip);
plot(rangeTested,bSlip,'r.','LineWidth',2);
plot(rangeTested([1,end]),[bSlipMean,bSlipMean]);
ylabel(['b_',i,' friction coeff']); 
xlabel('data set number'); 
legend('experiment value','mean')
title(['b_',i,' Friction']); 
%axis([-inf inf 0.0 1])
end
%% Estimate All Data

heightThreshold = 0.05+zTouch;

tBatch = [];
qBatch = [];
qdBatch = [];
qddBatch = [];

rangeTestedBatch = 2:numOfSetsNC;

for j =rangeTestedBatch

idx = find (zNC{j}<heightThreshold);
tBatchNew = (timeStepsNC{j}(idx))';
tBatch = [tBatch,tBatchNew];

qNew = (zNC{j}(idx))';
qBatch = [qBatch,qNew];

qdNew = (zdNC{j}(idx))';
qdBatch = [qdBatch,qdNew];

qddNew = (zddNC{j}(idx))';
qddBatch = [qddBatch,qddNew];
end

[bSurfaceEstBatch,frictionForceBatch,frictionForcehatBatch,rBatch,~] = ordinaryLeastSquaresNoContact(qdBatch, qddBatch, angleDeg, mdisc);

bSurfaceEstBatch

figure(50);clf; hold on;
plot(tBatch,qBatch,'k.',tBatch,qBatch,'k');
title('zBatch')
figure(51);clf; hold on;
plot(tBatch,qdBatch,'k.',tBatch,qdBatch,'k');
title('zdBatch')
figure(52);clf; hold on;
plot(tBatch,qddBatch,'k.',tBatch,qddBatch,'k');
title('zddBatch')
figure(53);clf; hold on;
plot(tBatch,frictionForceBatch,'k',tBatch,frictionForceBatch,'k.');
plot(tBatch,frictionForcehatBatch,'r',tBatch,frictionForcehatBatch,'r*');
h = legend('Friction Force Data','','Friction Force Estimated','');
title('forceBatch')
%set(h,'Interpreter','Latex');
figure(54)
plot(tBatch,rBatch,'k.',tBatch,rBatch,'k');
title('residualsBatch')

%% Test out the estimate
for j = rangeTestedBatch

t0 = timeStepsNC{j}(1);
tf = timeStepsNC{j}(end);
z0 = zNC{j}(1);
zd0= zdNC{j}(1);
m = mdisc;
%b = bsurfaceEst(j,1);
bsurface = bSurfaceEstBatch(1); %this estimate is used in the end
g = 9.81;

% syms z(t)
% Dz = diff(z);
% z(t) = dsolve(diff(z, t, t) == - b/m * Dz - g*sind(angleDeg), z(t0) == z0, Dz(t0) == zd0);
% z(t) = simplify(z)

%mDisc*z'' + 0.1728*z'  + 0.01043*sign(z') = 0.131*9.81*sin(36/180*pi), z(0) = 0.05309, z'(0)=1.668

zdA = @(t) (m*g/bsurface *sind(angleDeg)*(exp(-bsurface/m*(t-t0))-1) + zd0 * exp(-bsurface/m*(t-t0)));
figure(31)
tA = linspace(t0,tf,100);

zdAEval = zdA(tA);

plot(tA,zdAEval,'r','LineWidth',1)

end

  save('dataBSurface.mat', 'bsurface');%,'thetadfmincon');
else
  load('dataBSurface.mat');
end


numOfSets = size(zIC,1);


%% 1DOF Batch Estimate
if(calcOneDofProblem)
zNCBatch1d = [];
zdNCBatch1d = zNCBatch1d;
zddNCBatch1d = zNCBatch1d;
numOfSets = size(zIC,1);
rangeOfSets1DOF = 1:numOfSets;
%rangeOfSets1DOF = 2:2;

numOfSetsAdj1DOF = length(rangeOfSets1DOF);

for j = rangeOfSets1DOF
zNCBatch1d = [zNCBatch1d,zIC{j}'];
zdNCBatch1d = [zdNCBatch1d,zdIC{j}'];
zddNCBatch1d = [zddNCBatch1d,zddIC{j}'];
end

  rangeOfModels1d = 3:3;
  numOfModels1d = length(rangeOfModels1d);
  for j = 1:numOfModels1d
  l = rangeOfModels1d(j);
  Mb = l;
  Mk = l;
  [gammaBatch1d,WBatch1d] = softContactModel1D(zNCBatch1d, zdNCBatch1d, zddNCBatch1d, ...
    angleDeg, mdisc, bsurface, Mb, Mk);
   alpha1d = 0.05; % 95% confidence level

  [bBatch1d{j},bintBatch1d{j},rBatch1d{j},rintBatch1d{j},statsBatch1d(j,:)] = regress(gammaBatch1d,WBatch1d,alpha1d);
  mdl1d = fitlm(WBatch1d,gammaBatch1d);%,'Intercept',false) %model fit without intercept term
  %figure
  %plotResiduals(mdl1d)
  %mdl1dstep = step(mdl1d,'NSteps',20)
  %figure
  %plotResiduals(mdl1dstep)
  
  end
  
  figure(601); clf; hold on; 
  plot(rangeOfModels1d,statsBatch1d(:,1));  
  xlabel('model');title('Rsq statistics 1d');
  
  figure(602); clf; hold on; 
  plot(rangeOfModels1d,statsBatch1d(:,2));  
  xlabel('model');title('F stat 1d');

  figure(603); clf; hold on; 
  plot(rangeOfModels1d,statsBatch1d(:,3));  
  xlabel('model');title('p value 1d');

  figure(604); clf; hold on; 
  plot(rangeOfModels1d,statsBatch1d(:,4));  
  xlabel('model');title('error covariance 1d');

end

%% Estimate Theta
if(calcThetaFlag)
thetaIC0 = 0;
thetadIC0 = 0;

options = optimoptions(@fmincon);
options = optimoptions(options, 'SpecifyObjectiveGradient', true, 'Display', 'none');
lb = [-pi;-100];
ub = [pi;100];

thetaIC = cell(numOfSets,1);
%thetadfmincon = cell(numOfSets,1);
%thetaVec = zeros(size(z{1},1),2);
%theta = zeros(size(z{1},1),1);

  for j = 1:numOfSets
    numOfSamples = size(zIC{j},1);
    for k =1:numOfSamples
      %fun = @(thetaVec) resolveConstraintThetaThetaDotCostFun([thetaVec(1);0;z{j}(k)], [thetaVec(2);0;zd{j}(k)]);
      %[thetaVecTemp,fval] = fmincon(fun, [theta0; thetad0], [], [], [], [], lb, ub, [], options);
      %thetadfmincon{j}(k,1) = thetaVecTemp(2);
      fun = @(theta) resolveConstraintThetaCostFun([theta;0;zIC{j}(k)]);
      [thetaTemp,fval] = fmincon(fun, thetaIC0, [], [], [], [], lb(1), ub(1), [], options);
      
      thetaIC{j}(k,1) = thetaTemp;
      
      thetaIC0 = thetaTemp;
      %theta0 = thetaVecTemp(1);
      %thetad0 = thetaVecTemp(2);
  
    end
  end
  save('dataTheta.mat', 'timeStepsIC', 'zIC','zdIC','zddIC','thetaIC','numOfSets');%,'thetadfmincon');
else
  load('dataTheta.mat');
end

%% fit curves for theta
thetafitIC = cell( numOfSets, 1 );
thetaCmpIC = cell( numOfSets, 1 );
thetadIC = cell( numOfSets, 1 );
thetaddIC = cell( numOfSets, 1 );

gofTheta = struct( 'sse', cell( numOfSets, 1 ), ...
  'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );
ft = fittype( 'poly5' );

figure(500); clf; hold on;
figure(501); clf; hold on;
for j = 1: numOfSets

  figure(500);  hold on; plot(timeStepsIC{j}, thetaIC{j},'b*');
  [tData, thetaData] = prepareCurveData(  timeStepsIC{j}, thetaIC{j});
  
  plot(tData, thetaData,'r+');
  [thetafitIC{j}, gofTheta(j)] = fit( tData, thetaData, ft ,'Normalize','on');
  thetaCmpIC{j} = feval(thetafitIC{j},timeStepsIC{j});
  [thetadIC{j}, thetaddIC{j}] = differentiate(thetafitIC{j},timeStepsIC{j});

  figure(501);  
  p = plot(thetafitIC{j},tData,thetaData);%,[timeInterval{j}])      
  p(1).LineWidth = 2;
end


%Generate MORE DATA
timeSteps2 = cell( numOfSets, 1 );
z2 = cell( numOfSets, 1 );
zd2 = cell( numOfSets, 1 );
zdd2 = cell( numOfSets, 1 );
theta2 = cell( numOfSets, 1 );
thetad2 = cell( numOfSets, 1 );
thetadd2 = cell( numOfSets, 1 );
numOfLargeData = 100;

for j=1:numOfSets
timeSteps2{j} = linspace(timeStepsIC{j}(2),timeStepsIC{j}(end-1),numOfLargeData);
z2{j} = feval(zfitIC{j},timeSteps2{j});
[zd2{j}, zdd2{j}] = differentiate(zfitIC{j},timeSteps2{j});
theta2{j} = feval(thetafitIC{j},timeSteps2{j});
[thetad2{j}, thetadd2{j}] = differentiate(thetafitIC{j},timeSteps2{j});
end

%% Initial Parameter Guesses

%for dim =3
% q = [theta{j}';...
%     zeros(size(z{j}))';...
%     z{j}'];
% qd = [thetad{j}';...
%     zeros(size(zd{j}))';...
%     zd{j}'];
% qdd = [thetadd{j}';...
%     zeros(size(zdd{j}))';...  
%     zdd{j}'];

%for dim =2

rangeOfSets = 1:numOfSets;
rangeOfSets = 3:3;

numOfSetsAdj = length(rangeOfSets);

pEst = cell(numOfSetsAdj,1);

for i =1:numOfSetsAdj

j =  rangeOfSets(i);

% q = [theta2{j}';...
%     z2{j}'];
% qd = [thetad2{j}';...
%     zd2{j}'];
% qdd = [thetadd2{j}';...
%     zdd2{j}'];

q = [thetaIC{j}';...
    zIC{j}'];
qd = [thetadIC{j}';...
    zdIC{j}'];
qdd = [thetaddIC{j}';...
    zddIC{j}'];

if(estimateParamsFminConFlag)  
min = 0.00000001;
max = 100;
Ipulley = 0.00001798;
kpulley = 0.22;
bpulley = 0.0024;
% note: mdisc is not a parameter to be estimated

b0 = [Ipulley, kpulley, bpulley]; % simulation hand tuning
dimParams = size(b0,2);

fun = @(p) paramEstCostFun2D(p, q, qd, qdd, angleDeg, mdisc, bsurface);
options = optimoptions(@fmincon);
options = optimoptions(options, 'Display', 'iter');
 [pEst{i},fval] = fmincon(fun, b0, [],[], [], [], min*ones(1,dimParams), max*ones(1,dimParams),[], options);
 save('params.mat', 'pEstimated');
else
 rangeOfModels = 0:0;
  numOfModels = length(rangeOfModels);
  for j = 1:numOfModels
  l = rangeOfModels(j);
    Mb = l;
  Mk = l; 
 [gamma,W] = softContactModel2D(q, qd, qdd, angleDeg, mdisc, bsurface, Mb, Mk);
 alpha = 0.05; % 95% confidence level
 [b{i,j},bint{i,j},r{i,j},rint{i,j}] = regress(gamma,W,alpha);
 %load('params.mat');
  end
end

end
% 

I0Est= zeros(numOfSetsAdj,1);
IpulleyEstErr= zeros(numOfSetsAdj,2);

k0Est= zeros(numOfSetsAdj,1);
kpulleyEstErr= zeros(numOfSetsAdj,2);

b0Est= zeros(numOfSetsAdj,1);
bpulleyEstErr= zeros(numOfSetsAdj,2);


for i=1:numOfSetsAdj
    for j = 1:numOfModels
    l = rangeOfModels(j);
    I0Index = 1;
I0Est(i,j) = b{i,j}(I0Index);
I0EstErrLow(i,j) = bint{i,j}(I0Index,1)-b{i,j}(I0Index);
I0EstErrUp(i,j) = bint{i,j}(I0Index,2)-b{i,j}(I0Index);

b0Index = 2;
b0Est(i,j) = b{i,j}(b0Index);
b0EstErrLow(i,j) = b{i,j}(b0Index)-bint{i,j}(b0Index,1);
b0EstErrUp(i,j) = bint{i,j}(b0Index,2)-b{i,j}(b0Index);

k0Index = 3+l;
k0Est(i,j) = b{i,j}(k0Index);
k0EstErrLow(i,j) = b{i,j}(k0Index)-bint{i,j}(k0Index,1);
k0EstErrUp(i,j) = bint{i,j}(k0Index,2)-b{i,j}(k0Index);

end
end

rangeOfSetsMat = repmat(rangeOfSets',1,numOfModels);
rangeOfModelsCell = strcat({'Model '},int2str(rangeOfModels.')).';

figure(701); clf; hold on; 
errorbar(rangeOfSetsMat,I0Est,I0EstErrLow,I0EstErrUp,...
  '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',18); 
xlabel('data set number');title('I0');
legend(rangeOfModelsCell)

figure(702); clf; hold on; 
errorbar(rangeOfSetsMat,b0Est,b0EstErrLow,b0EstErrUp,...
  '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',18);  
xlabel('data set number');title('b0');
legend(rangeOfModelsCell)

figure(703); clf; hold on; 
errorbar(rangeOfSetsMat,k0Est,k0EstErrLow,k0EstErrUp,...
  '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',18); 
xlabel('data set number');title('k0');
legend(rangeOfModelsCell)


%% Batch Estimate
if(calcAllOfOneCombinedFlag)
qBatch = [];
qdBatch = qBatch;
qddBatch = qBatch;

for j =rangeOfSets
qNewBatch = [thetaIC{j}';...
    zIC{j}'];
qBatch = [qBatch,qNewBatch];

qdNewBatch = [thetadIC{j}';...
    zdIC{j}'];
qdBatch = [qdBatch,qdNewBatch];

qddNewBatch = [thetaddIC{j}';...
    zddIC{j}'];
qddBatch = [qddBatch,qddNewBatch];

end

if(estimateParamsFminConFlag)  

  %FMINCON
  fun = @(b) paramEstCostFun2D(b, qBatch, qdBatch, qddBatch, angleDeg, mdisc, bsurface);
  options = optimoptions(@fmincon);
  options = optimoptions(options, 'Display', 'iter');

  [pEstimatedAll,fval] = fmincon(fun, b0, [],[], [], [], ...
  min*ones(1,dimParams), max*ones(1,dimParams),[], options);
else
  rangeOfModels = 1:1;
  numOfModels = length(rangeOfModels);
  for j = 1:numOfModels
  l = rangeOfModels(j);
  Mb = l;
  Mk = l;
  [gammaBatch,WBatch] = softContactModel2D(qBatch, qdBatch, qddBatch, ...
    angleDeg, mdisc, bsurface,Mb,Mk);
   alpha = 0.05; % 95% confidence level

  [bBatch{j},bintBatch{j},rBatch{j},rintBatch{j},statsBatch(j,:)] = regress(gammaBatch,WBatch,alpha);
  mdl = fitlm(WBatch,gammaBatch)%,'Intercept',false) %model fit without intercept term
  %figure
  %plotResiduals(mdl)
  %mdl1 = step(mdl,'NSteps',20)
  %figure
  %plotResiduals(mdl1)
  

  end
  
  figure(801); clf; hold on; 
  plot(rangeOfModels,statsBatch(:,1));  
  xlabel('model');title('Rsq statistics');
  
  figure(802); clf; hold on; 
  plot(rangeOfModels,statsBatch(:,2));  
  xlabel('model');title('F stat');

  figure(803); clf; hold on; 
  plot(rangeOfModels,statsBatch(:,3));  
  xlabel('model');title('p value');

  figure(804); clf; hold on; 
  plot(rangeOfModels,statsBatch(:,4));  
  xlabel('model');title('error covariance');

end


end

end