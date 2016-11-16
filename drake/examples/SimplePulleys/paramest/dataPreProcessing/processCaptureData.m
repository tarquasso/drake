function [] = processCaptureData()
%% Parse Tension Experiment
filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_46_41';
%filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_48_35';
%filename = '~/soft_modeling_repo/dev/tracking/data/set6/16-May-2015 20_53_54.mat';

global generatePlot
generatePlot = true;

errStartTime = 5.955;
errEndTime = 5.963;

data.angleDeg = 36; % tilt of incline in degrees
data.mdisc = 0.131; %kg mass of disc
data.discRadius = 0.044230;

% I used a 250mm wand, but told the mpotive software that it is a 500mm
% wand, that is why we have a 1/2 factor here:
optiTrackWandErrorFactor = 1/2;

%Extract Start Time from Figues 1-3:
expStartTime = 5.383; %seconds 5.4
expEndTime = 8.051; %seconds 8.2

maxHeightOldFrame = 0.46;
touchPointOldFrame = 0.0279;
minHeightOldFrame = -0.03;

minHeight = minHeightOldFrame - touchPointOldFrame + data.discRadius;
maxHeight = maxHeightOldFrame - touchPointOldFrame + data.discRadius;
data.zTouch = data.discRadius;

load([filename,'.mat']);
%number of samples
numberOfSamples = History.i-1;
%Time values
timeStepsOrig = History.timestamps(1:numberOfSamples);
%height values
posDiscOrig = History.objectPosition(1:numberOfSamples,:); % x points up on the plane

posDiscScaled = optiTrackWandErrorFactor* posDiscOrig;


%% Plot the data Before Defining

yLabels = { 'height coordinate z [m]',...
            'horizontal coordinate x [m]',...
            'normal to plane coordinate y[m]'};
titleLabels = {'Check Z Coordinate of Data Set for limit values!',...
               'Check X Coordinate of Data Set for Skewedness of Data!',...
               'Check Y Coordinate of Data Set for Lifting off table!'};
for i = 1:3
figure(i); clf; plot(timeStepsOrig,posDiscScaled(:,i),'b'); hold on; xlabel('time [s]');
ylabel(yLabels{i}); title(titleLabels{i})
end


%Adjust Time Range and remove one error
indicesExperiment = find( timeStepsOrig > expStartTime & ...
  timeStepsOrig < expEndTime & ...
  (timeStepsOrig < errStartTime |...
  timeStepsOrig > errEndTime));
%adding first and last element to it - needed for data fitting
%indicesExperiment = [indicesExperiment(1)-1;indicesExperiment;indicesExperiment(end)+1];

%extract time and subtract start value
timeSteps = timeStepsOrig(indicesExperiment)-timeStepsOrig(indicesExperiment(1));
%Adjust order of position of Disc
posDisc = posDiscScaled(indicesExperiment,[2,3,1]);

%Adjust Height dimension according to touchPoint and Disc Radius
posDisc(:,3) = posDisc(:,3) - touchPointOldFrame + data.discRadius;

%% Plot the data After Defining
yLabels2 = { 'horizontal coordinate x [m]',...
            'normal to plane coordinate y[m]'...
            'height coordinate z [m]'};
          
titleLabels2 = {'Final X Coordinate of Data Set!',...
               'Final Y Coordinate of Data Set!',...
               'Final Z Coordinate of Data Set!'};
for i = 1:3
figure(i+10); clf; plot(timeSteps,posDisc(:,i),'b','LineWidth',0.6); hold on; xlabel('time [s]');
ylabel(yLabels2{i}); title(titleLabels2{i})
end
h2 = figure(13);
figure(302); clf; hold on

%% Extract each contact and each no_contact phase

idxNC = find(posDisc(:,3) > data.zTouch);
%idxNC = idxNC(2:end-1);
idxIC = find(posDisc(:,3) <= data.zTouch);
%idxIC = idxIC(1:end-1);

timeStepsNC = timeSteps(idxNC);
timeStepsIC = timeSteps(idxIC);
posDiscNC = posDisc(idxNC,:);
posDiscIC = posDisc(idxIC,:);

if(generatePlot)
%Add a line to the z coordinate figure
  figure(h2); hold on
  plot(timeStepsNC,posDiscNC(:,3),'g+','LineWidth',1.0)
  plot(timeSteps([1,end]),[data.zTouch,data.zTouch],'g','LineWidth',2)
  plot(timeStepsIC,posDiscIC(:,3),'r*','LineWidth',1.0)
  xlabel('time [s]')
  axis([-inf inf minHeight maxHeight])
% axis([expStartTime expEndTime minHeight maxHeight])
%   options.Format = 'eps';
%   hgexport(gcf,sprintf('plots/Set5_all.eps'),options);
end

offsetStep = 0; %defines additional points looked at before or after a contact data set

[numOfSetsNC,timeStepsSplitNC,posDiscSplitNC] = extractSets(idxNC,timeSteps,posDisc,offsetStep);
[numOfSetsIC,timeStepsSplitIC,posDiscSplitIC] = extractSets(idxIC,timeSteps,posDisc,offsetStep);

if(generatePlot)
figure(303);clf; hold on;
xlabel('time [s]')
ylabel('zd ')
title('velocities zd')

figure(304);clf; hold on;
xlabel('time [s]')
ylabel('zdd ')
title('accelerations zdd')
end

ftNC = fittype( 'poly3' );
[timeStepsExpandedNC,zNC,zdNC,zddNC,zfitNC] = fitCurves(numOfSetsNC,timeStepsSplitNC, posDiscSplitNC,data.zTouch,offsetStep,ftNC);
ftIC = fittype( 'poly5' );

[timeStepsExpandedIC,zIC,zdIC,zddIC,zfitIC] = fitCurves(numOfSetsIC,timeStepsSplitIC, posDiscSplitIC,data.zTouch,offsetStep,ftIC);

% % resave the data set
% newFilename = [filename,'_preprocessed.mat'];
% save(newFilename,'data');
end

function [numOfSets,timeStepsSplit,posDiscSplit] = extractSets(idxMode,timeSteps, posDisc,offsetStep)
global generatePlot
%% Define points when in contact and points when not in contact
boolTransitions = diff(idxMode)> 1;
%last step shortens it by one value, so for the start points, add 1 logical in front:
boolTransitionsStarts = [true;boolTransitions];
idxStart =  idxMode(boolTransitionsStarts);
boolTransitionsEnds = [boolTransitions;true];
idxEnd = idxMode(boolTransitionsEnds);
numOfSets = sum(boolTransitionsStarts);

idxExtended = cell(numOfSets,1);
timeStepsSplit = cell(numOfSets,1);
posDiscSplit = cell(numOfSets,1);


for j = 1:numOfSets
  idxExtended{j} = (idxStart(j)-offsetStep):(idxEnd(j)+offsetStep);
  timeStepsSplit{j} = timeSteps(idxExtended{j});
  posDiscSplit{j} = posDisc(idxExtended{j},:);
%   tICES{j} = tICE{j}-tICE{j}(1);
  if(generatePlot)
    figure(302);
    plot(timeStepsSplit{j}(1),posDiscSplit{j}(1,3),'m*','LineWidth',4.5)
    plot(timeStepsSplit{j}(end),posDiscSplit{j}(end,3),'k*','LineWidth',4.5)
  end
end

end

function [timeStepsExpanded,z,zd,zdd,zfit] = fitCurves(numOfSets,timeSteps, posDisc,zTouch,offsetStep,ft)

global generatePlot

%% fit curves
zfit = cell( numOfSets, 1 );
z = cell( numOfSets, 1 );
zd = cell( numOfSets, 1 );
zdd = cell( numOfSets, 1 );

gof = struct( 'sse', cell( numOfSets, 1 ), ...
     'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );
%ft = fittype( 'poly2' );
%ft = fittype( 'cubicinterp' );


timeStepsExpanded = cell( numOfSets, 1 );

elementsToCheck = 10000;


for j = 1: numOfSets
  %figure(301+10*j); clf; hold on; plot(timeSteps{j}, posDisc{j}(:,3),'*');
  [tData, zData] = prepareCurveData(  timeSteps{j}, posDisc{j}(:,3) );
  %figure(302+10*j); clf; hold on; plot(tData, zData,'*');
  
  %   tMin = min(tData);
  %   tMax = max(tData);
  tMean = mean(tData);
  %tDataAdj = 2*(tData-tMean)/(tMax-tMin);
  tData = tData-tMean;
  %zMean = mean(zData);
  zMean = 0;
  zData = zData - zMean;
  
  [zfit{j}, gof(j)] = fit( tData, zData, ft );
%   tEval = linspace(tData(1),tData(2),elementsToCheck);
%   fZeroPotentials = feval(zfit{j},tEval)+zMean;
%   [fZeroFirst,idx] = min(abs(fZeroPotentials-zTouch));
%   tFirst = tEval(idx);
%   
%   tEval = linspace(tData(end-1),tData(end),elementsToCheck);
%   fZeroPotentials = feval(zfit{j},tEval)+zMean;
%   [fZeroLast,idx] = min(abs(fZeroPotentials-zTouch));
%   tLast = tEval(idx);
  
  %timeStepsExpanded{j} = [tFirst;tData(1+offsetStep:end-offsetStep);tLast];
  timeStepsExpanded{j} = tData;
  timeStepsExpanded{j} = linspace(tData(1)-0.05,tData(end)+0.05,elementsToCheck);
  
  %tEval  = linspace(timeSteps{j}(1),timeSteps{j}(end),elementsToCheck);
  
  z{j} = feval(zfit{j},timeStepsExpanded{j});
  z{j} = z{j} + zMean;
  
  [zd{j}, zdd{j}] = differentiate(zfit{j},timeStepsExpanded{j});
  timeStepsExpanded{j} = timeStepsExpanded{j} + tMean;
  
  if(generatePlot)
    figure(302)
%     p1 = plot(zfit{j},timeStepsExpanded{j},z{j});%,[timeInterval{j}])
%     p1(1).LineWidth = 2;

    p1 = plot(timeStepsExpanded{j},z{j});
    p1(1).LineWidth = 2;
    xlabel('time [s]')
    ylabel('zd ')
    title('positions z')

    
    figure(303)
    p2 = plot(timeStepsExpanded{j},zd{j});
    p2(1).LineWidth = 2;
    xlabel('time [s]')
    ylabel('zd ')
    title('velocities zd')
    
    figure(304)
    p3 = plot(timeStepsExpanded{j},zdd{j});
    p3(1).LineWidth = 2;
  end
end


end