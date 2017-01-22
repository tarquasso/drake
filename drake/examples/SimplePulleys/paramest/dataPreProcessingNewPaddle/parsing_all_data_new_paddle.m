%% Parse in all data files 4-6

subset = 'A';

angleDeg = 35.8; % tilt of incline in degrees
mdisc = 0.110; %kg mass of disc
discRadius = 0.0877/2;
spread=0.542;
%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5.955;
errEndTime = 5.955;

% I used a 250mm wand and told the motive software that it is a 250mm
% wand, that is why we have a 1 factor here:
optiTrackWandScalingFactor = 1;

maxHeightAfterScaling = 0.5519;
ox = 0.447480;
rx = 0.029;
oz = -0.0659;
rz = +0.02075;
touchPointAfterScaling = oz+rz+discRadius; %oz+rz
staticSpringStrectchingPointAfterScaling = -0.00506;
minHeightAfterScaling = -0.085;
horizOffset = ox-rx-spread/2;

%% SET 10 - A
%clear all
close all
clc
dataSetFolder = 'set10';
name = 'Juggler-Experiment_2017-01-19_15-59-05';
filename = ['~/Dropbox (MIT)/Robotics Research/soft_modeling_db/Paramater_Estimation_RSS_2017/',dataSetFolder,'/',name];
dataSetName = [dataSetFolder,subset];

%Extract Start Time from Figues 1-3:
expStartTime = 0.875; %seconds 5.4
expEndTime = 16.71; %8.943; %seconds 16.71

processCaptureData_new_paddle(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName,horizOffset);


%% SET 11
%clear all
close all
clc
dataSetFolder = 'set11';
name = 'Juggler-Experiment_2017-01-19_16-03-32';
filename = ['~/Dropbox (MIT)/Robotics Research/soft_modeling_db/Paramater_Estimation_RSS_2017/',dataSetFolder,'/',name];
dataSetName = [dataSetFolder,subset];

%Extract Start Time from Figues 1-3:
expStartTime = 0.925; %seconds 2.392
expEndTime = 12.5; %8.943; %seconds 16.71

processCaptureData_new_paddle(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName,horizOffset);

%% SET 12
%clear all
close all
clc
dataSetFolder = 'set12';
name = 'Juggler-Experiment_2017-01-19_16-04-44';
filename = ['~/Dropbox (MIT)/Robotics Research/soft_modeling_db/Paramater_Estimation_RSS_2017/',dataSetFolder,'/',name];
subset = 'A';
dataSetName = [dataSetFolder,subset];

%Extract Start Time from Figues 1-3:
expStartTime = 0.95; %seconds 2.392
expEndTime = 14.28; %8.943; %seconds 16.71

processCaptureData_new_paddle(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName,horizOffset);

%% SET 13
%clear all
close all
clc
dataSetFolder = 'set13';
name = 'Juggler-Experiment_2017-01-19_16-05-25';
filename = ['~/Dropbox (MIT)/Robotics Research/soft_modeling_db/Paramater_Estimation_RSS_2017/',dataSetFolder,'/',name];
subset = 'A';
dataSetName = [dataSetFolder,subset];

%Extract Start Time from Figues 1-3:
expStartTime = 0.825; %seconds 2.392
expEndTime = 8.1; %8.943; %seconds 16.71

processCaptureData_new_paddle(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName,horizOffset);