global generatePlot

dataSetFolder = 'set1';
subset = 'A';
dataSetName = [dataSetFolder,subset];
name = 'syntheticPaddleDataWithAccel';
filename = ['~/soft_modeling_repo/dev/simulation/',dataSetFolder,'/',name];
filenameWithExtension = [filename,'.mat'];
load(filenameWithExtension);

[pathstr,name,ext] = fileparts(filenameWithExtension);
newFilename = [filename,'_preprocessed.mat'];


angleDeg = 36; % tilt of incline in degrees
mdisc = 0.131; %kg mass of disc
spread=0.29;


discRadius = 0.044230;

generatePlot = true;

%Truncate data (remove last just contact part)
tf = 2.135;
tf = 1.765;
idxUsed = find(t < tf);
%number of samples
numberOfSamples = length(idxUsed);
timeStepsTrimmed = t(idxUsed);
xTrimmed = x(:,idxUsed);
mdTrimmed = gradient(xTrimmed(1,:))./gradient(timeStepsTrimmed);
xdTrimmed = [mdTrimmed;xd(:,idxUsed)];

zTouch = discRadius; %touchpoint is measured based on the center of the disc


%% Plot the data Before Defining
if(generatePlot)

  touchPoint = discRadius;
  staticSpringStrectchingPoint = discRadius;

  yLabels = { 'angle theta[rad]',...
    'horizontal coordinate x [m]',...
    'height coordinate z [m]'};
  titleLabels = {'Sim Data Theta',...
    'Sim Data X',...
    'Sim Data Z'};
  for i = 1:3 %only plot z axis
    figure(i); clf; 
    plot(timeStepsTrimmed,xTrimmed(i+1,:),'-.ob'); 
    hold on; xlabel('time [s]');
    if(i == 3) %for z axis
      plot(timeStepsTrimmed(1,[1,end]),[touchPoint,touchPoint],'g','LineWidth',2)
      plot(timeStepsTrimmed(1,[1,end]),[staticSpringStrectchingPoint,staticSpringStrectchingPoint],'y','LineWidth',2)  
    end
    
    ylabel(yLabels{i}); title(titleLabels{i})
  end
end

% %Adjust Time Range and remove one error
% indicesExperiment = find( timeStepsOrig > expStartTime & ...
%   timeStepsOrig < expEndTime & ...
%   (timeStepsOrig < errStartTime |...
%   timeStepsOrig > errEndTime));
% 
% % adding first and last element to it - needed for data fitting
% indicesExperiment = [indicesExperiment(1)-1;indicesExperiment;indicesExperiment(end)+1];

%extract time and subtract start value
timeSteps = timeStepsTrimmed-timeStepsTrimmed(1);
% 
% %% Plot the data After Defining
% if(generatePlot)
%   yLabels2 = { 'horizontal coordinate x [m]',...
%     'normal to plane coordinate y[m]'...
%     'height coordinate z [m]'};
%   
%   titleLabels2 = {'X Coordinate of Data Set',...
%     'Y Coordinate of Data Set',...
%     'Z Coordinate of Data Set'};
%   for i = 1:3
%     figure(i+10); clf; 
%     plot(timeSteps,posDisc(:,i),'-.or','LineWidth',0.6); hold on; xlabel('time [s]');
%     ylabel(yLabels2{i}); title(titleLabels2{i})
%     if(i == 3) %for z axis
%       plot(timeSteps([1,end]),[zTouch,zTouch],'g','LineWidth',2)
%     end
%     title([titleLabels2{i},' (',dataSetName,')'])
%     options.Format = 'eps';
%     hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',titleLabels2{i},'.eps'],options);
%   end
%   
% end

%% Get acceleration

dt = gradient(timeSteps);
dt1 = diff(timeSteps);
dt1 = [dt1(1),dt1];
figure(87); clf;
plot(dt1,'r.')
hold on
grid on
plot(dt,'b.')

%xdTrimmed = gradient(xTrimmed)./dt;
xdTrimmedGrad = gradient(xTrimmed)./dt;
xdTrimmedDiff = diff(xTrimmed,1,2);
xdTrimmedDiff = [xdTrimmedDiff(:,1),xdTrimmedDiff];
xdTrimmedDiff = xdTrimmedDiff./dt;
figure(84); clf;
plot((xTrimmed(1,:)),'m-..')
hold on
grid on
plot(xTrimmed(5,:),'r-.*')
plot(xdTrimmed(2,:),'k-.^')

plot(xdTrimmedGrad(2,:),'b-.o')

plot(xdTrimmedDiff(2,:),'g-.+')

legend('mode','x(5,:)','xd(2,:)','gradient','diff')
title('Velocities Tehta - Different Methods')

for i=1:3

figure(85+i); clf;
plot((xTrimmed(1,:)),'m-..')
hold on
grid on
plot(xdTrimmed(4+i,:),'k-.^')
plot(xdTrimmedGrad(4+i,:),'b-.o')
plot(xdTrimmedDiff(4+i,:),'g-.+')
legend('mode','xd','gradient','diff')
title(['Accelerations ','xd(',num2str(4+i),')',' Comparision- Different Methods'])
end

%% Extract each contact and each no_contact phase

%% Separate flight and contact
modeNC = 1;
modeIC = 2;
idxNC = find (xTrimmed(1,:) == modeNC);
idxIC = find (xTrimmed(1,:) == modeIC);
timeStepsSplitNC = timeSteps(idxNC);
timeStepsSplitIC = timeSteps(idxIC);
xTrimmedNC = xTrimmed(:,idxNC);
xTrimmedIC = xTrimmed(:,idxIC);


if(generatePlot)
  %Add a line to the z coordinate
  figure(21); clf; hold on
  plot(timeSteps,xTrimmed(4,:),'g','LineWidth',1.0);
  plot(timeStepsSplitNC,xTrimmedNC(4,:),'b.','LineWidth',2.0)
  plot(timeSteps([1,end]),[zTouch,zTouch],'g','LineWidth',2)
  plot(timeStepsSplitIC,xTrimmedIC(4,:),'r.','LineWidth',2.0)
  xlabel('time [s]')
  %axis([-inf inf minHeight maxHeight])
  title(['Height z - Unseparated (',dataSetName,')'])
  typeofPlot = 'z';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
end

%% 
if(generatePlot)
  
  figure(22); clf; hold on
  plot(timeStepsSplitNC,xTrimmedNC(4,:),'g.','LineWidth',3.0)
  plot(timeSteps(1,[1,end]),[zTouch,zTouch],'g','LineWidth',1.5)
  plot(timeStepsSplitIC,xTrimmedIC(4,:),'r.','LineWidth',3.0)
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
  
  
  figure(32); clf; hold on
  plot(timeStepsSplitNC,xTrimmedNC(2,:),'g.','LineWidth',3.0)
  plot(timeStepsSplitIC,xTrimmedIC(2,:),'r.','LineWidth',3.0)
  %axis([-inf inf minHeight maxHeight])
  title(['Theta - Separated (',dataSetName,')'])
  xlabel('time [s]')
  ylabel('Angle theta')
  
  figure(33);clf; hold on;
  xlabel('time [s]')
  ylabel('thetad')
  title(['Angular Velocity thetad',' (',dataSetName,')'])
  
  figure(34);clf; hold on;
  xlabel('time [s]')
  ylabel('thetadd')
  title(['Angular Acceleration thetadd',' (',dataSetName,')'])
  
  figure(42); clf; hold on
  plot(timeStepsSplitNC,xTrimmedNC(3,:),'g.','LineWidth',3.0)
  plot(timeStepsSplitIC,xTrimmedIC(3,:),'r.','LineWidth',3.0)
  %axis([-inf inf minHeight maxHeight])
  title(['X - Separated (',dataSetName,')'])
  xlabel('time [s]')
  ylabel('Pos x')
  
  figure(43);clf; hold on;
  xlabel('time [s]')
  ylabel('xd')
  title(['Velocity xd',' (',dataSetName,')'])
  
  figure(44);clf; hold on;
  xlabel('time [s]')
  ylabel('xdd')
  title(['Acceleration xdd',' (',dataSetName,')'])
end

typeNC = 'NoSplit';
minDataPointsNC = 5;
[numOfSetsNC,timeStepsSplitNC,xTrimmedSplitNC,xdTrimmedSplitNC] = extractSetsSim(idxNC,timeSteps,xTrimmed,xdTrimmed,minDataPointsNC,typeNC);

typeIC = 'NoSplit';
minDataPointsIC = 5;
[numOfSetsIC,timeStepsSplitIC,xTrimmedSplitIC,xdTrimmedSplitIC] = extractSetsSim(idxIC,timeSteps,xTrimmed,xdTrimmed,minDataPointsIC,typeIC);


[timeStepsNC,thetaNC,thetadNC,thetaddNC,xNC,xdNC,xddNC,zNC,zdNC,zddNC] = ...
  extractIndividualStates(numOfSetsNC,timeStepsSplitNC, xTrimmedSplitNC,xdTrimmedSplitNC);

[timeStepsIC,thetaIC,thetadIC,thetaddIC,xIC,xdIC,xddIC,zIC,zdIC,zddIC] = ...
  extractIndividualStates(numOfSetsIC,timeStepsSplitIC, xTrimmedSplitIC,xdTrimmedSplitIC);

save(newFilename,'angleDeg','mdisc','spread','zTouch',...
   'timeStepsNC','thetaNC','thetadNC','thetaddNC','xNC','xdNC','xddNC','zNC','zdNC','zddNC',...
   'timeStepsIC','thetaIC','thetadIC','thetaddIC','xIC','xdIC','xddIC','zIC','zdIC','zddIC');

  