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


% filename = '~/soft_modeling_repo/dev/tracking/data/set6/16-May-2015 20_53_54.mat';
% tp = 0.0558;
% expStartTime = 0; %seconds 5.4 
% expEndTime = 6.7; %seconds 8.2


generatePlot = true; 
% [times, z,zd,zdd, tICE, tICES, xICE, zICE, numOfSets, qfit, gof] = ...
%   parseTensionExperimentData(filename,tp,expStartTime,expEndTime,optiTrackWandErrorFactor,minHeight,maxHeight,generatePlot);

%save('data.mat', 'times', 'z','zd','zdd')

load('data.mat')
% fit it to a polynomial to extract q, qd, qdd

%j=1;
%m = length(z{j});

%q = [zeros(m,2);z{1}];
theta0 = 0;
thetad0 = -0;
z{1} = z{1}+0.044230;
options = optimoptions(@fmincon);
options = optimoptions(options, 'SpecifyObjectiveGradient', true, 'Display', 'none');
 lb = [-pi;-100];
 ub = [pi;100];
 numSets = size(z,1);
%thetaVec = cell(numSets,1);
  thetaVec = zeros(size(z{1},1),2);
for k =1:size(z{1},1)
fun = @(thetaVec) resolveConstraintThetaThetaDotCostFun([thetaVec(1);0;z{1}(k)], [thetaVec(2);0;zd{1}(k)]);
[thetaVec(k,:),fval] = fmincon(fun, [theta0; thetad0], [], [], [], [], lb, ub, [], options);
theta0 = thetaVec(k,1);
thetad0 = thetaVec(k,2);
end

 %  [x,fval] = fmincon(fun, theta, [], [], [], [], lb(1), ub(1), [], options);

% qd = 10*rand(10,3);
% qdd = 100*rand(100,3);


% p0 = rand(4,1); % simulation hand tuning

%p0 = 



% fun = @(p) paramEstCostFun(p, q, qd, qdd);
% options = optimoptions(@fmincon);
% options = optimoptions(options, 'Display', 'iter');
% 
% [x,fval] = fmincon(fun, p0, [],[], [], [], 0.01*ones(4,1), 1000*ones(4,1),[], options);