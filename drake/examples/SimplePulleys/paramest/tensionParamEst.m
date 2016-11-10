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
% [times, z,zd,zdd, tICE, tICES, xICE, zICE, numOfSets, qfit, gof] = ...
%   parseTensionExperimentData(filename,tp,expStartTime,expEndTime,optiTrackWandErrorFactor,minHeight,maxHeight,generatePlot);

%save('data.mat', 'times', 'z','zd','zdd')

load('data.mat')


%j=1;
%m = length(z{j});

%q = [zeros(m,2);z{1}];
theta0 = 0;
thetad0 = -0;

z{1} = z{1} + discRadius; % shift the z coordinate by the disc radius up

options = optimoptions(@fmincon);
options = optimoptions(options, 'SpecifyObjectiveGradient', true, 'Display', 'none');
 lb = [-pi;-100];
 ub = [pi;100];
 numSets = size(z,1);
%thetaVec = cell(numSets,1);
  thetaVec = zeros(size(z{1},1),2);
%theta = zeros(size(z{1},1),1);
for k =1:size(z{1},1)
fun = @(thetaVec) resolveConstraintThetaThetaDotCostFun([thetaVec(1);0;z{1}(k)], [thetaVec(2);0;zd{1}(k)]);
[thetaVec(k,:),fval] = fmincon(fun, [theta0; thetad0], [], [], [], [], lb, ub, [], options);

% fun = @(theta) resolveConstraintThetaCostFun([theta;0;z{1}(k)]);
%  [theta(k),fval] = fmincon(fun, theta0, [], [], [], [], lb(1), ub(1), [], options);

theta0 = thetaVec(k,1);
thetad0 = thetaVec(k,2);
end

%% Initial Parameter Guesses
min = 0.00000001;
max = 100;

Ipulley = 0.00001798;
kpulley = 0.22;
bpulley = 0.0024;
% note: mdisc is not a parameter to be estimated

p0 = [Ipulley;kpulley;bpulley]; % simulation hand tuning
dimParams = length(p0);


fun = @(p) paramEstCostFun(p, q, qd, qdd, angleDeg, mdisc);
options = optimoptions(@fmincon);
options = optimoptions(options, 'Display', 'iter');

[pEstimated,fval] = fmincon(fun, p0, [],[], [], [], ...
           min*ones(dimParams,1), max*ones(dimParams,1),[], options);
         
IpulleyEst = pEstimated(1);
kpulleyEst = pEstimated(2);
bpulleyEst = pEstimated(3);

IpulleyEst,kpulleyEst,bpulleyEst