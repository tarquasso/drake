

%% SET 5 - A
clear all
close all
clc
dataSetName = 'set5A';
name = '16-May-2015 20_46_41';
filename = ['~/soft_modeling_repo/dev/tracking/data/',dataSetName,'/',name];

%Extract Start Time from Figues 1-3:
expStartTime = 5.383; %seconds 5.4
expEndTime = 8.051; %seconds 8.2

angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
discRadius = 0.044230;
spread=0.29;

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5.955;
errEndTime = 5.963;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandScalingFactor = 1/2;

maxHeightAfterScaling = 0.46;
touchPointAfterScaling = 0.0558/2;
staticSpringStrectchingPointAfterScaling = 0.0528/2;
minHeightAfterScaling = -0.03;

processCaptureData(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName);


%% SET 5 - B
clear all
close all
clc
dataSetName = 'set5B';
name = '16-May-2015 20_48_35';
filename = ['~/soft_modeling_repo/dev/tracking/data/',dataSetName,'/',name];

angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
discRadius = 0.044230;
spread=0.29;

%Extract Start Time from Figues 1-3:
expStartTime = 4.128;
expEndTime = 6.753; 

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5.955;
errEndTime = 5.955;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandScalingFactor = 1/2;

maxHeightAfterScaling = 0.429;
touchPointAfterScaling = 0.0558/2;
staticSpringStrectchingPointAfterScaling = 0.0528/2;
minHeightAfterScaling = -0.025;

processCaptureData(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName);

%% SET 6 - a

clear all
close all
clc
dataSetName = 'set6A';
name = '16-May-2015 20_52_26';
filename = ['~/soft_modeling_repo/dev/tracking/data/',dataSetName,'/',name];

% Following values same for sets 5 and 6
angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
discRadius = 0.044230;
spread=0.38;

%These values change:
%Extract Start Time from Figues 1-3:
expStartTime = 5.459;
expEndTime = 7.425; 

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5;
errEndTime = 5;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandScalingFactor = 1/2;

maxHeightAfterScaling = 0.444;
touchPointAfterScaling = 0.0558/2; %fixed
staticSpringStrectchingPointAfterScaling = 0.0528/2;
minHeightAfterScaling = -0.015;


processCaptureData(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName);


%% SET 6 - b

clear all
close all
clc
dataSetName = 'set6B';
name = '16-May-2015 20_53_54';
filename = ['~/soft_modeling_repo/dev/tracking/data/',dataSetName,'/',name];

angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
discRadius = 0.044230;
spread=0.38;

%Extract Start Time from Figues 1-3:
expStartTime = 4.421;
expEndTime = 6.393; 

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5;
errEndTime = 5;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandScalingFactor = 1/2;

maxHeightAfterScaling = 0.444;
touchPointAfterScaling = 0.0558/2; %fixed
staticSpringStrectchingPointAfterScaling = 0.0528/2;
minHeightAfterScaling = -0.015;

processCaptureData(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName);

%% SET 4 - A
clear all
close all
clc

dataSetName = 'set4A';
name = '16-May-2015 20_22_45';
filename = ['~/soft_modeling_repo/dev/tracking/data/',dataSetName,'/',name];

%Extract Start Time from Figues 1-3:
expStartTime = 5.319; %seconds 5.4
expEndTime = 8.076; %seconds 8.2

angleDeg = 36; % tilt of incline in degrees
mdisc = 0.0693; %kg mass of disc
discRadius = 0.044230;
spread=0.29;

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5.0;
errEndTime = 5.0;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandScalingFactor = 1/2;

maxHeightAfterScaling = 0.45;
touchPointAfterScaling = 0.0558/2; %value already scaled compareed to experiment notes
staticSpringStrectchingPointAfterScaling = 0.0528/2;
minHeightAfterScaling = -0.01;

processCaptureData(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName);

%% SET 4 - B
clear all
close all
clc
dataSetName = 'set4B';
name = '16-May-2015 20_23_46';
filename = ['~/soft_modeling_repo/dev/tracking/data/',dataSetName,'/',name];

%Extract Start Time from Figure 1
expStartTime = 4.571;
expEndTime = 7.295; 

angleDeg = 36; % tilt of incline in degrees
mdisc = 0.0693; %kg mass of disc
discRadius = 0.044230;
spread=0.29;

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5.0;
errEndTime = 5.0;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandScalingFactor = 1/2;

maxHeightAfterScaling = 0.45;
touchPointAfterScaling = 0.0558/2; %value already scaled compareed to experiment notes
staticSpringStrectchingPointAfterScaling = 0.0528/2;
minHeightAfterScaling = -0.01;

processCaptureData(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName);