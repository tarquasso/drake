function [] = processCaptureData(filename,expStartTime,expEndTime,maxHeightOldFrame,touchPointOldFrame,minHeightOldFrame,angleDeg,mdisc,discRadius,errStartTime,errEndTime,optiTrackWandErrorFactor)
%% Parse Tension Experiment
%%%%%%%%%%%%%%%%%%
global generatePlot
generatePlot = true;

minHeight = minHeightOldFrame - touchPointOldFrame + discRadius;
maxHeight = maxHeightOldFrame - touchPointOldFrame + discRadius;
zTouch = discRadius;

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
if(i==3)
plot(timeStepsOrig([1,end]),[zTouch,zTouch],'g','LineWidth',2)
end
ylabel(yLabels{i}); title(titleLabels{i})
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
posDisc(:,3) = posDisc(:,3) - touchPointOldFrame + discRadius;

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


%% Extract each contact and each no_contact phase

idxNC = find(posDisc(:,3) > zTouch);
idxNC = idxNC(2:end-1);
idxIC = find(posDisc(:,3) <= zTouch);
idxIC = idxIC(1:end-1);

timeStepsNC = timeSteps(idxNC);
timeStepsIC = timeSteps(idxIC);
posDiscNC = posDisc(idxNC,:);
posDiscIC = posDisc(idxIC,:);

if(generatePlot)
%Add a line to the z coordinate figuretimeStepsNC,zNC,zdNC,zddNC,timeStepsLargeNC,zLargeNC,zdLargeNC,zddLargeNC,zfitNC
  figure(h2); hold on
  plot(timeStepsNC,posDiscNC(:,3),'g+','LineWidth',1.0)
  plot(timeSteps([1,end]),[zTouch,zTouch],'g','LineWidth',2)
  plot(timeStepsIC,posDiscIC(:,3),'r*','LineWidth',1.0)
  xlabel('time [s]')
  axis([-inf inf minHeight maxHeight])
% axis([expStartTime expEndTime minHeight maxHeight])
%   options.Format = 'eps';
%   hgexport(gcf,sprintf('plots/Set5_all.eps'),options);

figure(302); clf; hold on
  plot(timeStepsNC,posDiscNC(:,3),'g+','LineWidth',1.0)
  plot(timeSteps([1,end]),[zTouch,zTouch],'g','LineWidth',2)
  plot(timeStepsIC,posDiscIC(:,3),'r*','LineWidth',1.0)
  xlabel('time [s]')
  axis([-inf inf minHeight maxHeight])
end

offsetStepNC = 0; %defines additional points looked at before or after a contact data set
offsetStepIC = 1;
[numOfSetsNC,timeStepsSplitNC,posDiscSplitNC] = extractSets(idxNC,timeSteps,posDisc,offsetStepNC);
[numOfSetsIC,timeStepsSplitIC,posDiscSplitIC] = extractSets(idxIC,timeSteps,posDisc,offsetStepIC);

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
zdd_lbNC = -inf;
zdd_ubNC = 0.0;
[timeStepsNC,zNC,zdNC,zddNC,timeStepsLargeNC,zLargeNC,zdLargeNC,zddLargeNC,zfitNC] = fitCurves(numOfSetsNC,timeStepsSplitNC, posDiscSplitNC,zTouch,offsetStepNC,ftNC,zdd_lbNC,zdd_ubNC);



%% Setting up the fitting for the contact phase
ftIC = fittype( 'poly4' );
zdd_lbIC = -sind(angleDeg)*9.81;
zdd_ubIC = inf;
[timeStepsIC,zIC,zdIC,zddIC,timeStepsLargeIC,zLargeIC,zdLargeIC,zddLargeIC,zfitIC] = fitCurves(numOfSetsIC,timeStepsSplitIC, posDiscSplitIC,zTouch,offsetStepIC,ftIC,zdd_lbIC,zdd_ubIC);
 
% resave the data set
newFilename = [filename,'_preprocessed.mat'];
save(newFilename,'angleDeg','mdisc','zTouch',...
  'timeStepsNC','zNC','zdNC','zddNC','timeStepsLargeNC',...
  'zLargeNC','zdLargeNC','zddLargeNC','zfitNC','timeStepsIC','zIC',...
  'zdIC','zddIC','timeStepsLargeIC','zLargeIC','zdLargeIC','zddLargeIC','zfitIC');

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

function [timeStepsExpandedUseful,z,zd,zdd,timeStepsLarge,zLarge,zdLarge,zddLarge,zfit] = fitCurves(numOfSets,timeSteps, posDisc,zTouch,offsetStep,ft,zdd_lb,zdd_ub)

global generatePlot

%% fit curves
zfit = cell( numOfSets, 1 );
timeStepsExpanded = cell( numOfSets, 1 );
timeStepsExpandedUseful= cell( numOfSets, 1 );
z = cell( numOfSets, 1 );
zd = cell( numOfSets, 1 );
zdd = cell( numOfSets, 1 );
timeStepsLarge = cell( numOfSets, 1 );
zLarge = cell( numOfSets, 1 );
zdLarge = cell( numOfSets, 1 );
zddLarge = cell( numOfSets, 1 );

gof = struct( 'sse', cell( numOfSets, 1 ), ...
     'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );
%ft = fittype( 'poly2' );
%ft = fittype( 'cubicinterp' );

for j = 1: numOfSets
  %figure(301+10*j); clf; hold on; plot(timeSteps{j}, posDisc{j}(:,3),'*');
  [tData, zData] = prepareCurveData(  timeSteps{j}, posDisc{j}(:,3) );
  %figure(302+10*j); clf; hold on; plot(tData, zData,'*');
  
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
  z{j} = z{j};% + zMean;
  
  [zd{j}, zdd{j}] = differentiate(zfit{j},timeStepsExpanded{j});
  
  % scale the 
  timeStepsExpanded{j} = timeStepsExpanded{j};% + tMean;
  
  
  idxUseful = find(zdd{j} > zdd_lb & zdd{j} < zdd_ub);
  
  timeStepsExpandedUseful{j} = timeStepsExpanded{j}(idxUseful);
  z{j} = z{j}(idxUseful);
  zd{j} = zd{j}(idxUseful);
  zdd{j} = zdd{j}(idxUseful);
  
  if(generatePlot)
    figure(302)
%     p1 = plot(zfit{j},timeStepsExpanded{j},z{j});%,[timeInterval{j}])
%     p1(1).LineWidth = 2;

    p1 = plot(timeStepsExpandedUseful{j},z{j});
    p1(1).LineWidth = 2;
    xlabel('time [s]')
    ylabel('zd ')
    title('positions z')

    
    figure(303)
    p2 = plot(timeStepsExpandedUseful{j},zd{j});
    p2(1).LineWidth = 2;
    xlabel('time [s]')
    ylabel('zd ')
    title('velocities zd')
    
    figure(304)
    p3 = plot(timeStepsExpandedUseful{j},zdd{j});
    p3(1).LineWidth = 2;
  end
  
  %% Gernerate more data
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