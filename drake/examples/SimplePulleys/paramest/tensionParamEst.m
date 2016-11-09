%% Parse in Optitrack Capture Data

%% Parse Tension Experiment

% So far only working for Data Set 5
filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_46_41.mat';
%filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_48_35.mat';

% Set 5 Touching point: 55.8mm - manually measured from the Optitrack

%Static spring stretching point: 52.8 mm
expStartTime = 5.35; %seconds 5.4 
expEndTime = 8; %seconds 8.2

optiTrackWandErrorFactor = 1/2;

tp = 0.0558 * optiTrackWandErrorFactor;

minHeight = -0.03;
maxHeight = 0.46;


% filename = '~/soft_modeling_repo/dev/tracking/data/set6/16-May-2015 20_53_54.mat';
% tp = 0.0558;
% expStartTime = 0; %seconds 5.4 
% expEndTime = 6.7; %seconds 8.2


generatePlot = true; 
[xuls,tuls,numOfSets] = parseTensionExperimentData(filename,tp,expStartTime,expEndTime,optiTrackWandErrorFactor,minHeight,maxHeight,generatePlot);

xuls,tuls,numOfSets

%% Fit Polynomial


% adjust in height to the tensionWParamsExp.urdf coordinates

% fit it to a polynomial to extract q, qd, qdd

% 
% q = rand(10,3);
% qd = 10*rand(10,3);
% qdd = 100*rand(100,3);


% p0 = rand(4,1); % simulation hand tuning

%p0 = 



% fun = @(p) paramEstCostFun(p, q, qd, qdd);
% options = optimoptions(@fmincon);
% options = optimoptions(options, 'Display', 'iter');
% 
% [x,fval] = fmincon(fun, p0, [],[], [], [], 0.01*ones(4,1), 1000*ones(4,1),[], options);