

%% SET 5 - A
clear all
close all
clc
filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_46_41';

%Extract Start Time from Figues 1-3:
expStartTime = 5.383; %seconds 5.4
expEndTime = 8.051; %seconds 8.2

angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
discRadius = 0.044230;

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5.955;
errEndTime = 5.963;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandErrorFactor = 1/2;

maxHeightOldFrame = 0.46;
touchPointOldFrame = 0.0279;
minHeightOldFrame = -0.03;

processCaptureData(filename,expStartTime,expEndTime,maxHeightOldFrame,touchPointOldFrame,minHeightOldFrame,angleDeg,mdisc,discRadius,errStartTime,errEndTime,optiTrackWandErrorFactor)


%% SET 5 - B
clear all
close all
clc
filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_48_35';

angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
discRadius = 0.044230;

%Extract Start Time from Figues 1-3:
expStartTime = 4.128;
expEndTime = 6.75; 

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5.955;
errEndTime = 5.955;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandErrorFactor = 1/2;

maxHeightOldFrame = 0.429;
touchPointOldFrame = 0.0279;
minHeightOldFrame = -0.025;

processCaptureData(filename,expStartTime,expEndTime,maxHeightOldFrame,touchPointOldFrame,minHeightOldFrame,angleDeg,mdisc,discRadius,errStartTime,errEndTime,optiTrackWandErrorFactor)

%% SET 6 - a

clear all
close all
clc
filename = '~/soft_modeling_repo/dev/tracking/data/set6/16-May-2015 20_53_54';


angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
discRadius = 0.044230;

%Extract Start Time from Figues 1-3:
expStartTime = 4.421;
expEndTime = 6.58; 

%if there is an error, surround it by this time frae. if not, just make
%both values equal:
errStartTime = 5;
errEndTime = 5;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandErrorFactor = 1/2;

maxHeightOldFrame = 0.444;
touchPointOldFrame = 0.0279; %fixed
minHeightOldFrame = -0.015;

processCaptureData(filename,expStartTime,expEndTime,maxHeightOldFrame,touchPointOldFrame,minHeightOldFrame,angleDeg,mdisc,discRadius,errStartTime,errEndTime,optiTrackWandErrorFactor)


