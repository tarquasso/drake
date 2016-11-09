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
[times, z,zd,zdd, tICE, tICES, xICE, zICE, numOfSets, qfit, gof] = ...
  parseTensionExperimentData(filename,tp,expStartTime,expEndTime,optiTrackWandErrorFactor,minHeight,maxHeight,generatePlot);


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