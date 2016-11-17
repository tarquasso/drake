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
calcThetaFlag = true; % calculating Thetas
estimateParamsFminConFlag = false; %using fmin con to estimate the parameters for each contact phase individually
calcAllOfOneCombinedFlag = false; % using fmincon to estimate the parameters for all contact phases as one data set


% timeStepsNC,zNC,zdNC,zddNC,timeStepsLargeNC,zLargeNC,zdLargeNC,zddLargeNC,zfitNC


% generatePlot = true;

%% Estimate bsurface

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
plot(t,force{j},'k',t,force{j},'k.');
plot(t,forcehat{j},'r',t,forcehat{j},'r*');
h = legend('$f$','$f$','$\hat{f}$','$\hat{f}$');
set(h,'Interpreter','Latex');
figure(41)
plot(t,r{j},'k.');
plot(t,r{j},'k');

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

rangeTested = 1:numOfSetsNC;

for j =rangeTested

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

[bSurfaceEst,frictionForceBatch,frictionForcehatBatch,rBatch,~] = ordinaryLeastSquaresNoContact(qdBatch, qddBatch, angleDeg, mdisc);

bSurfaceEst

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
for j = rangeTested


t0 = timeStepsNC{j}(1);
tf = timeStepsNC{j}(end);
z0 = zNC{j}(1);
zd0= zdNC{j}(1);
m = mdisc;
b = bsurfaceEst(j,1);
g = 9.81;

% syms z(t)
% Dz = diff(z);
% z(t) = dsolve(diff(z, t, t) == - b/m * Dz - g*sind(angleDeg), z(t0) == z0, Dz(t0) == zd0);
% z(t) = simplify(z)

%mDisc*z'' + 0.1728*z'  + 0.01043*sign(z') = 0.131*9.81*sin(36/180*pi), z(0) = 0.05309, z'(0)=1.668

zdA = @(t) (m*g/b *sind(angleDeg)*(exp(-b/m*(t-t0))-1) + zd0 * exp(-b/m*(t-t0)));
figure(31)
tA = linspace(t0,tf,100);

zAEval = zdA(tA);

plot(tA,zAEval,'r','LineWidth',3)

end
return;

%% 
theta0 = 0;
thetad0 = 0;
numOfSets = size(z,1);

options = optimoptions(@fmincon);
options = optimoptions(options, 'SpecifyObjectiveGradient', true, 'Display', 'none');
lb = [-pi;-100];
ub = [pi;100];

theta = cell(numOfSets,1);
thetadfmincon = cell(numOfSets,1);
%thetaVec = zeros(size(z{1},1),2);
%theta = zeros(size(z{1},1),1);

if(calcThetaFlag)
  for j = 1:numOfSets
    for k =1:size(z{j},1)
      %fun = @(thetaVec) resolveConstraintThetaThetaDotCostFun([thetaVec(1);0;z{j}(k)], [thetaVec(2);0;zd{j}(k)]);
      %[thetaVecTemp,fval] = fmincon(fun, [theta0; thetad0], [], [], [], [], lb, ub, [], options);
      %thetadfmincon{j}(k,1) = thetaVecTemp(2);
      fun = @(theta) resolveConstraintThetaCostFun([theta;0;z{j}(k)]);
      [thetaTemp,fval] = fmincon(fun, theta0, [], [], [], [], lb(1), ub(1), [], options);
      
      theta{j}(k,1) = thetaTemp;
      
      theta0 = thetaTemp;
      %theta0 = thetaVecTemp(1);
      %thetad0 = thetaVecTemp(2);
  
    end
  end
  save('data2.mat', 'timeSteps', 'z','zd','zdd','theta','numOfSets','thetadfmincon');
else
  load('data2.mat');
end

%% fit curves for theta
thetafit = cell( numOfSets, 1 );
thetaCmp = cell( numOfSets, 1 );
thetad = cell( numOfSets, 1 );
thetadd = cell( numOfSets, 1 );

gofTheta = struct( 'sse', cell( numOfSets, 1 ), ...
  'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );
%ft = fittype( 'poly2' );
ft = fittype( 'cubicinterp' );

figure(500); clf; hold on;
figure(510); clf; hold on;
for j = 1: numOfSets

  figure(500);  hold on; plot(timeSteps{j}, theta{j},'b*');
  [tData, thetaData] = prepareCurveData(  timeSteps{j}, theta{j});
  
  plot(tData, thetaData,'r+');
  [thetafit{j}, gofTheta(j)] = fit( tData, thetaData, ft );
  thetaCmp{j} = feval(thetafit{j},timeSteps{j});
  [thetad{j}, thetadd{j}] = differentiate(thetafit{j},timeSteps{j});

  figure(510);  
  p = plot(thetafit{j},tData,thetaData);%,[timeInterval{j}])      
  p(1).LineWidth = 2;
end


%MORE DATA
timeSteps2 = cell( numOfSets, 1 );
z2 = cell( numOfSets, 1 );
zd2 = cell( numOfSets, 1 );
zdd2 = cell( numOfSets, 1 );
theta2 = cell( numOfSets, 1 );
thetad2 = cell( numOfSets, 1 );
thetadd2 = cell( numOfSets, 1 );
numOfLargeData = 101;

for j=1:numOfSets
timeSteps2{j} = linspace(timeSteps{j}(2),timeSteps{j}(end-1),numOfLargeData);
z2{j} = feval(zfit{j},timeSteps2{j});
[zd2{j}, zdd2{j}] = differentiate(zfit{j},timeSteps2{j});
theta2{j} = feval(thetafit{j},timeSteps2{j});
[thetad2{j}, thetadd2{j}] = differentiate(thetafit{j},timeSteps2{j});
end

%% Initial Parameter Guesses
min = 0.00000001;
max = 100;

Ipulley = 0.00001798;
kpulley = 0.22;
bpulley = 0.0024;
bsurface = 0.01;
% note: mdisc is not a parameter to be estimated

p0 = [Ipulley, kpulley, bpulley, bsurface]; % simulation hand tuning
dimParams = size(p0,2);

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

pEstimated = cell(numOfSets,1);

for j =1:numOfSets
  
q = [theta2{j}';...
    z2{j}'];
qd = [thetad2{j}';...
    zd2{j}'];
qdd = [thetadd2{j}';...
    zdd2{j}'];

fun = @(p) paramEstCostFun2D(p, q, qd, qdd, angleDeg, mdisc);
options = optimoptions(@fmincon);
options = optimoptions(options, 'Display', 'iter');
if(estimateParamsFminConFlag)  
 [pEstimated{j},fval] = fmincon(fun, p0, [],[], [], [], min*ones(1,dimParams), max*ones(1,dimParams),[], options);
else
 pEstimated{j} = ordinaryLeastSquares(q, qd, qdd, angleDeg, mdisc);
end

end

if(estimateParamsFminConFlag)  
save('params.mat', 'pEstimated');
else
%load('params.mat');
end
IpulleyEst= zeros(numOfSets,1);
kpulleyEst= zeros(numOfSets,1);
bpulleyEst= zeros(numOfSets,1);
bsurfaceEst = zeros(numOfSets,1);

for i=1:numOfSets
IpulleyEst(i) = pEstimated{i}(1);
kpulleyEst(i) = pEstimated{i}(2);
bpulleyEst(i) = pEstimated{i}(3);
bsurfaceEst(i) = pEstimated{i}(4);
end

figure(701); clf; hold on; plot(IpulleyEst); xlabel('data set number');title('IpulleyEst');
figure(702); clf; hold on; plot(kpulleyEst); xlabel('data set number');title('kpulleyEst');
figure(703); clf; hold on; plot(kpulleyEst); xlabel('data set number');title('bpulleyEst');
figure(704); clf; hold on; plot(bsurfaceEst); xlabel('data set number');title('bsurfaceEst');

if(calcAllOfOneCombinedFlag)
q = [];
qd = q;
qdd = q;

for j =1:numOfSets
qNew = [theta{j}';...
    z{j}'];
q = [q,qNew];

qdNew = [thetad{j}';...
    zd{j}'];
qd = [qd,qdNew];

qddNew = [thetadd{j}';...
    zdd{j}'];
qdd = [qdd,qddNew];
end

fun = @(p) paramEstCostFun2D(p, q, qd, qdd, angleDeg, mdisc);
options = optimoptions(@fmincon);
options = optimoptions(options, 'Display', 'iter');

[pEstimatedAll,fval] = fmincon(fun, p0, [],[], [], [], ...
  min*ones(1,dimParams), max*ones(1,dimParams),[], options);
end

end