%% Set the following parameters to false after running it for the first time:

parseExpFlag = true;

calcThetaFlag = true;

estimateParamsFminConFlag = false;

calcAllOfOneCombinedFlag = false;

%% Parse in Optitrack Capture Data

%% Parse Tension Experiment

% So far only working for Data Set 5
filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_46_41.mat';
%filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_48_35.mat';

load(filename)
%number of samples
nos = History.i-1;
timeVals = History.timestamps(1:nos);
zVals = History.objectPosition(1:nos,1); % x points up on the plane
xVals = History.objectPosition(1:nos,2);
yVals = History.objectPosition(1:nos,3);

figure(10); clf; plot(timeVals,zVals,'b'); hold on; xlabel('time [s]');
ylabel('height coordinate z [m]'); title('Check Z Coordinate of Data Set for limit values!')

figure(11); clf; plot(timeVals,xVals,'b'); hold on; xlabel('time [s]');
ylabel('horizontal coordinate x [m]'); title('Check X Coordinate of Data Set for Skewedness of Data!')

%% Fill in observed data of that data setS
% Set 5 Touching point: 55.8mm - manually measured from the Optitrack

%Static spring stretching point: 52.8 mm
expStartTime = 5.35; %seconds 5.4
expEndTime = 8; %seconds 8.2

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandErrorFactor = 1/2;

tp = 0.0558 * optiTrackWandErrorFactor;

minHeight = -0.03;
maxHeight = 0.46;

angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
discRadius = 0.044230;

% filename = '~/soft_modeling_repo/dev/tracking/data/set6/16-May-2015 20_53_54.mat';
% tp = 0.0558;
% expStartTime = 0; %seconds 5.4
% expEndTime = 6.7; %seconds 8.2



generatePlot = true;


if(parseExpFlag)
  
  [timeSteps, z,zd,zdd, tICE, tICES, xICE, zICE, numOfSets, zfit, gofTheta] = ...
    parseTensionExperimentData(filename,tp,discRadius,expStartTime,expEndTime,optiTrackWandErrorFactor,minHeight,maxHeight,generatePlot);
  
  save('data.mat', 'times', 'z','zd','zdd');
else
  load('data.mat');
end

%j=1;
%m = length(z{j});

%q = [zeros(m,2);z{1}];
theta0 = 0;
thetad0 = -0;
numOfSets = size(z,1);

%numOfSets = 1; %HERRRRRRE

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
  save('data2.mat', 'times', 'z','zd','zdd','theta','numOfSets','thetadfmincon');
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


for j = 1: numOfSets

  figure(500+j); clf; hold on; plot(timeSteps{j}, theta{j},'b*');
  [tData, thetaData] = prepareCurveData(  timeSteps{j}, theta{j});
  
  plot(tData, thetaData,'r+');
  [thetafit{j}, gofTheta(j)] = fit( tData, thetaData, ft );
  thetaCmp{j} = feval(thetafit{j},timeSteps{j});
  [thetad{j}, thetadd{j}] = differentiate(thetafit{j},timeSteps{j});

  figure(510+j)
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