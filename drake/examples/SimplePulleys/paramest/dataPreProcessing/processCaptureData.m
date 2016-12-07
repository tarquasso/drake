function [] = processCaptureData(filename,expStartTime,expEndTime,maxHeightAfterScaling,...
  touchPointAfterScaling,staticSpringStrectchingPointAfterScaling,...
  minHeightAfterScaling,angleDeg,mdisc,discRadius,spread,...
  errStartTime,errEndTime,optiTrackWandScalingFactor,dataSetName)
%% Parse Tension Experiment
%%%%%%%%%%%%%%%%%%
global generatePlot
generatePlot = true;
windowFit = false;

filenameWithExtension = [filename,'.mat'];
load(filenameWithExtension);
[pathstr,name,ext] = fileparts(filenameWithExtension);

%number of samples
numberOfSamples = History.i-1;
%Time values
timeStepsOrig = History.timestamps(1:numberOfSamples);
%height values
posDiscOrig = History.objectPosition(1:numberOfSamples,:); % x points up on the plane

%multiply by factor of 1/2 to fix the data scaling
posDiscScaled = optiTrackWandScalingFactor * posDiscOrig;

zTouch = discRadius; %touchpoint is measured based on the center of the disc

%% Plot the data Before Defining
if(generatePlot)
  yLabels = { 'height coordinate z [m]',...
    'horizontal coordinate x [m]',...
    'normal to plane coordinate y[m]'};
  titleLabels = {'Unformatted Data - Check Z Coordinate of Data Set for limit values!',...
    'Unformatted Data - Check X Coordinate of Data Set for Skewedness of Data!',...
    'Unformatted Data - Check Y Coordinate of Data Set for Lifting off table!'};
  for i = 1:1 %only plot z axis
    figure(i); clf; plot(timeStepsOrig,posDiscScaled(:,i),'b'); hold on; xlabel('time [s]');
    if(i == 1) %for z axis
      plot(timeStepsOrig([1,end]),[touchPointAfterScaling,touchPointAfterScaling],'g','LineWidth',2)
      plot(timeStepsOrig([1,end]),[staticSpringStrectchingPointAfterScaling,staticSpringStrectchingPointAfterScaling],'y','LineWidth',2)
      
    end
    ylabel(yLabels{i}); title(titleLabels{i})
  end
end

%Adjust Time Range and remove one error
indicesExperiment = find( timeStepsOrig > expStartTime & ...
  timeStepsOrig < expEndTime & ...
  (timeStepsOrig < errStartTime |...
  timeStepsOrig > errEndTime));

% adding first and last element to it - needed for data fitting
indicesExperiment = [indicesExperiment(1)-1;indicesExperiment;indicesExperiment(end)+1];

%extract time and subtract start value
timeSteps = timeStepsOrig(indicesExperiment)-timeStepsOrig(indicesExperiment(1));

%Adjust order of position of Disc
posDisc = posDiscScaled(indicesExperiment,[2,3,1]);

%Adjust Height dimension according to touchPoint and Disc Radius
posDisc(:,3) = posDisc(:,3) - touchPointAfterScaling + discRadius;

%% Plot the data After Defining
if(generatePlot)
  yLabels2 = { 'horizontal coordinate x [m]',...
    'normal to plane coordinate y[m]'...
    'height coordinate z [m]'};
  
  titleLabels2 = {'X Coordinate of Data Set',...
    'Y Coordinate of Data Set',...
    'Z Coordinate of Data Set'};
  for i = 1:3
    figure(i+10); clf; plot(timeSteps,posDisc(:,i),'b','LineWidth',0.6); hold on; xlabel('time [s]');
    ylabel(yLabels2{i}); title(titleLabels2{i})
    if(i == 3) %for z axis
      plot(timeSteps([1,end]),[zTouch,zTouch],'g','LineWidth',2)
    end
    title([titleLabels2{i},' (',dataSetName,')'])
    options.Format = 'eps';
    hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',titleLabels2{i},'.eps'],options);
  end
  
end

%% Extract each contact and each no_contact phase

idxNC = find(posDisc(:,3) > zTouch);
idxNC = idxNC(2:end-1);
  
idxIC = find(posDisc(:,3) <= zTouch);
idxIC = idxIC(1:end-1);

timeStepsNC = timeSteps(idxNC);
timeStepsIC = timeSteps(idxIC);
posDiscNC = posDisc(idxNC,:);
posDiscIC = posDisc(idxIC,:);

%minHeight = minHeightAfterScaling - touchPointAfterScaling + discRadius;
%maxHeight = maxHeightAfterScaling - touchPointAfterScaling + discRadius;

if(generatePlot)
  %Add a line to the z coordinate
  figure(21); clf; hold on
  plot(timeSteps,posDisc(:,i),'g','LineWidth',1.0);
  plot(timeStepsNC,posDiscNC(:,3),'b.','LineWidth',2.0)
  plot(timeSteps([1,end]),[zTouch,zTouch],'g','LineWidth',2)
  plot(timeStepsIC,posDiscIC(:,3),'r.','LineWidth',2.0)
  xlabel('time [s]')
  %axis([-inf inf minHeight maxHeight])
  title(['Height z - Unseparated (',dataSetName,')'])
  typeofPlot = 'z';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
end

if(generatePlot)
  
  figure(22); clf; hold on
  plot(timeStepsNC,posDiscNC(:,3),'g.','LineWidth',3.0)
  plot(timeSteps([1,end]),[zTouch,zTouch],'g','LineWidth',1.5)
  plot(timeStepsIC,posDiscIC(:,3),'r.','LineWidth',3.0)
  %axis([-inf inf minHeight maxHeight])
  title(['Height z - Separated (',dataSetName,')'])
  xlabel('time [s]')
  ylabel('z')
  
  figure(23);clf; hold on;
  xlabel('time [s]')
  ylabel('zd')
  title(['Velocity zd',' (',dataSetName,')'])
  
  figure(24);clf; hold on;
  xlabel('time [s]')
  ylabel('zdd')
  title(['Acceleration zdd',' (',dataSetName,')'])
  
end

typeNC = 'NoSplit';
offsetStepNC = 0; %defines additional points looked at before or after a contact data set
minDataPointsNC = 8;
[numOfSetsNC,timeStepsSplitNC,posDiscSplitNC] = extractSets(idxNC,timeSteps,posDisc,offsetStepNC,minDataPointsNC,typeNC);

typeIC = 'NoSplit';
offsetStepIC = 1; % taking one additional data point at start and end of the data set
minDataPointsIC = 5;
[numOfSetsIC,timeStepsSplitIC,posDiscSplitIC] = extractSets(idxIC,timeSteps,posDisc,offsetStepIC,minDataPointsIC,typeIC);

if(windowFit)
%% Setting up the fitting for the non contact phase
[timeStepsNC,zNC,zdNC,zddNC] ...
  = fitCurvesSlidingWindow(numOfSetsNC,timeStepsSplitNC, posDiscSplitNC);
%% Setting up the fitting for the contact phase
[timeStepsIC,zIC,zdIC,zddIC] ...
  = fitCurvesSlidingWindow(numOfSetsIC,timeStepsSplitIC, posDiscSplitIC);
else
  %% Setting up the fitting for the non contact phase
ftNC = fittype( 'poly4' ); %results in a linear model for the acceleration change due to sliding friction
zdd_lbNC = -inf;
zdd_ubNC = 0.0;
[timeStepsNC,zNC,zdNC,zddNC,timeStepsLargeNC,zLargeNC,zdLargeNC,zddLargeNC,zfitNC] ...
  = fitCurves(numOfSetsNC,timeStepsSplitNC, posDiscSplitNC,offsetStepNC,ftNC,zdd_lbNC,zdd_ubNC);

%% Setting up the fitting for the contact phase
ftIC = fittype( 'poly5' );
zdd_lbIC = -sind(angleDeg)*9.81*1/2; %half the gravitational force because we do not know the sliding friction yet
%zdd_lbIC = -inf; %half the gravitational force because we do not know the sliding friction yet
zdd_ubIC = inf;
[timeStepsIC,zIC,zdIC,zddIC,timeStepsLargeIC,zLargeIC,zdLargeIC,zddLargeIC,zfitIC] ...
  = fitCurves(numOfSetsIC,timeStepsSplitIC, posDiscSplitIC,offsetStepIC,ftIC,zdd_lbIC,zdd_ubIC);
end

%% more plotting
if(generatePlot)
  figure(22)
  typeofPlot = 'z';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  figure(23)
  typeofPlot = 'zd';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  figure(24)
  typeofPlot = 'zdd';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
end


% resave the data set
newFilename = [filename,'_preprocessed.mat'];
save(newFilename,'angleDeg','mdisc','spread','zTouch',...
  'timeStepsNC','zNC','zdNC','zddNC','zfitNC',...
  'timeStepsIC','zIC','zdIC','zddIC','zfitIC');%,...
  %'timeStepsLargeNC','zLargeNC','zdLargeNC','zddLargeNC','timeStepsLargeIC','zLargeIC','zdLargeIC','zddLargeIC');

end

function [numOfSetsNew,timeStepsSplit,posDiscSplit] = extractSets(idxMode,timeSteps, posDisc,offsetStep,minDataPoints,type)
global generatePlot
%% Define points when in contact and points when not in contact
boolTransitions = diff(idxMode)> 1;
%last step shortens it by one value, so for the start points, add 1 logical in front:
boolTransitionsStarts = [true;boolTransitions];
idxStart =  idxMode(boolTransitionsStarts);
boolTransitionsEnds = [boolTransitions;true];
idxEnd = idxMode(boolTransitionsEnds);

numOfSets = length(idxEnd);%sum(boolTransitionsStarts);

if(strcmp(type,'Split')) % no contact
  
idxStartNew = [];
idxEndNew = [];
for l = 1:numOfSets
  idxCheck = (idxStart(l)):(idxEnd(l));
  zDataRelevant = posDisc(idxCheck,3);
  
  [~,I] = max(zDataRelevant);
  
  if(I==1)
    idxEndNew = [idxEndNew,idxEnd(l)];
    idxStartNew = [idxStartNew,idxStart(l)];
  else
    idxEndNew = [idxEndNew,idxCheck(I),idxEnd(l)];
    idxStartNew = [idxStartNew,idxStart(l),idxCheck(I)];
    
  end
end

else
    idxStartNew = idxStart;
    idxEndNew = idxEnd;
end
  
%Remove last sets if contain to little data:
keepGoing = true;
while(keepGoing)
if (idxEndNew(end)-idxStartNew(end)<minDataPoints) %+1 so it accounts for the apex point we do not account for
  %remove the last data set:
  idxStartNew = idxStartNew(1:end-1);
  idxEndNew = idxEndNew(1:end-1);
else
  keepGoing = false;
end
end

numOfSetsNew = length(idxEndNew);%sum(boolTransitionsStarts);

idxExtended = cell(numOfSetsNew,1);
timeStepsSplit = cell(numOfSetsNew,1);
posDiscSplit = cell(numOfSetsNew,1);


for j = 1:numOfSetsNew
  idxExtended{j} = (idxStartNew(j)-offsetStep):(idxEndNew(j)+offsetStep);
  timeStepsSplit{j} = timeSteps(idxExtended{j});
  posDiscSplit{j} = posDisc(idxExtended{j},:);
   
  if(generatePlot)
    figure(22);
    plot(timeStepsSplit{j}(1),posDiscSplit{j}(1,3),'m*','LineWidth',4.5)
    plot(timeStepsSplit{j}(end),posDiscSplit{j}(end,3),'k*','LineWidth',4.5)
  end
end

end

function [timeStepsExpandedUseful,z,zd,zdd,timeStepsLarge,zLarge,zdLarge,zddLarge,zfit] = ...
  fitCurves(numOfSets,timeSteps, posDisc,offsetStep,ft,zdd_lb,zdd_ub)

global generatePlot

%% fit curves
zfit = cell( numOfSets, 1 );
timeStepsExpanded = cell( numOfSets, 1 );
timeStepsExpandedUseful= cell( numOfSets, 1 );
z = cell( numOfSets, 1 );
z_pred = cell( numOfSets, 1 );
zd = cell( numOfSets, 1 );
zdd = cell( numOfSets, 1 );
timeStepsLarge = cell( numOfSets, 1 );
zLarge = cell( numOfSets, 1 );
zdLarge = cell( numOfSets, 1 );
zddLarge = cell( numOfSets, 1 );

gof = struct( 'sse', cell( numOfSets, 1 ), ...
  'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );

for j = 1: numOfSets
  %figure(21+10*j); clf; hold on; plot(timeSteps{j}, posDisc{j}(:,3),'*');
  [tData, zData] = prepareCurveData(  timeSteps{j}, posDisc{j}(:,3) );
  %figure(22+10*j); clf; hold on; plot(tData, zData,'*');
  
  %   %   tMin = min(tData);
  %   %   tMax = max(tData);
  %   tMean = mean(tData);
  %   tMean = 0;
  %   %tDataAdj = 2*(tData-tMean)/(tMax-tMin);
  %   tData = tData-tMean;
  %   %zMean = mean(zData);
  %   zMean = 0;
  %   zData = zData - zMean;
  
  [zfit{j}, gof(j)] = fit( tData, zData, ft ,'Normalize','on');
  %   tEval = linspace(tData(1),tData(2),elementsToCheck);
  %   fZeroPotentials = feval(zfit{j},tEval)+zMean;
  %   [fZeroFirst,idx] = min(abs(fZeroPotentials-zTouch));
  %   tFirst = tEval(idx);
  %
  %   tEval = linspace(tData(end-1),tData(end),elementsToCheck);
  %   fZeroPotentials = feval(zfit{j},tEval)+zMean;
  %   [fZeroLast,idx] = min(abs(fZeroPotentials-zTouch));
  %   tLast = tEval(idx);
  
  %leave out the last datapoint that was from the pure sliding phase:
  timeStepsExpanded{j} = tData(1+offsetStep:end-offsetStep);
  
  % include sliding phase point:
  %timeStepsExpanded{j} = tData;
  
  %Add an expander that allows to see more of the fitting over the
  %boundaries:
  %expander = 0.0;
  %timeStepsExpanded{j} = linspace(tData(1)-expander,tData(end)+expander,elementsToCheck);
  
  % Take a lot of data points...
  %timeStepsExpanded{j}  = linspace(timeStepsExpanded{j}(1),timeStepsExpanded{j}(end),elementsToCheck);
  
  z{j} = feval(zfit{j},timeStepsExpanded{j});
  z_pred{j} = predint(zfit{j},timeStepsExpanded{j},0.95,'observation','off');

  [zd{j}, zdd{j}] = differentiate(zfit{j},timeStepsExpanded{j});
    
  idxUseful = find(zdd{j} > zdd_lb & zdd{j} < zdd_ub);
    
  timeStepsExpandedUseful{j} = timeStepsExpanded{j}(idxUseful);
  z{j} = z{j}(idxUseful);
  zd{j} = zd{j}(idxUseful);
  zdd{j} = zdd{j}(idxUseful);
  
  if(generatePlot)
    figure(22);
    p1 = plot(timeStepsExpandedUseful{j},z{j});
    %     p1 = plot(zfit{j},timeStepsExpanded{j},z{j});%,[timeInterval{j}])\
    p1(1).LineWidth = 1;
    plot(timeStepsExpanded{j},z_pred{j},'r');
    %plot(zfit{j},'predobs');
        
    figure(23)
    p2 = plot(timeStepsExpandedUseful{j},zd{j},'.');
    p2(1).LineWidth = 1;
    plot(timeStepsExpandedUseful{j},zd{j});
    
    figure(24)
    p3 = plot(timeStepsExpandedUseful{j},zdd{j},'.');
    p3(1).LineWidth = 1;
    plot(timeStepsExpandedUseful{j},zdd{j});
  end
  
  %% Generate more data
  elementsToCheck = size(timeStepsExpanded{j},1)*10;
  
  timeStepsLarge{j}  = linspace(timeStepsExpanded{j}(1),timeStepsExpanded{j}(end),elementsToCheck);
  zLarge{j} = feval(zfit{j},timeStepsLarge{j});
  [zdLarge{j}, zddLarge{j}] = differentiate(zfit{j},timeStepsLarge{j});
  
  idxUseful = find(zddLarge{j} > zdd_lb & zddLarge{j} < zdd_ub);
  
  timeStepsLarge{j} = timeStepsLarge{j}(idxUseful);
  zLarge{j} = zLarge{j}(idxUseful);
  zdLarge{j} = zdLarge{j}(idxUseful);
  zddLarge{j} = zddLarge{j}(idxUseful);
  
end
end

function [timeSteps,z,zd,zdd] = ...
  fitCurvesSlidingWindow(numOfSets,timeSteps, posDisc)

global generatePlot

%% fit curves
zfit = cell( numOfSets, 1 );
z = cell( numOfSets, 1 );
zpred = cell( numOfSets, 1 );
zd = cell( numOfSets, 1 );
zdd = cell( numOfSets, 1 );
gof = struct( 'sse', cell( numOfSets, 1 ), ...
  'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );
ft = fittype( 'poly2' );

for j = 1: numOfSets
  numOfSteps = length( timeSteps{j});
  for k = 1:(numOfSteps)
    indices = max(1,k-3):1:min(numOfSteps,k+3);
    timeStepsWindow =  timeSteps{j}(indices);
    zDataWindow = posDisc{j}(indices,3);
    [tData, zData] = prepareCurveData(timeStepsWindow, zDataWindow );
    
    [zfit{j}, gof(j)] = fit( tData, zData, ft ,'Normalize','on');
    z{j}(k) = feval(zfit{j},timeSteps{j}(k));
    zpredLocal = predint(zfit{j},timeSteps{j}(k), 0.95,'functional','off');
    zpred{j}(k,:) = zpredLocal;
    
    [zd{j}(k), zdd{j}(k)] = differentiate(zfit{j},timeSteps{j}(k));
  end
  
  if(generatePlot)
    figure(22);
    p1 = plot(timeSteps{j},z{j});
    p1(1).LineWidth = 1;
    %plot the predicition bounds:
    plot(timeSteps{j},zpred{j},'r');

    figure(23)
    p2 = plot(timeSteps{j},zd{j},'.');
    p2(1).LineWidth = 1;
    plot(timeSteps{j},zd{j});
    
    figure(24)
    p3 = plot(timeSteps{j},zdd{j},'.');
    p3(1).LineWidth = 1;
    plot(timeSteps{j},zdd{j});
  end
  
end
end
