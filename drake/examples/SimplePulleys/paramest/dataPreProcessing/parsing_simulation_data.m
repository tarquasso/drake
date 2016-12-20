global generatePlot

dataSetFolder = 'set2';
subset = 'A';
dataSetName = [dataSetFolder,subset];
name = 'syntheticPaddleDataWithAccel';
% paramsUsed.I = 0.00001798;
% paramsUsed.b= 0.001;
% paramsUsed.bSurface = 0.3;
% paramsUsed.k = 4;

paramsUsed.I = 0.000107687954594736;
paramsUsed.b= 0.000550304675835062;
paramsUsed.bSurface = 0.213679574645237;
paramsUsed.k = 0.797496299490406;

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
tf = 2.52;
%tf = 1.765;
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
  staticSpringStretchingPoint = discRadius;
  
  yLabels = { 'angle theta[rad]',...
    'horizontal coordinate x [m]',...
    'height coordinate z [m]'};
  titleLabels = {'Sim Data Theta',...
    'Sim Data X',...
    'Sim Data Z'};
  for i = 1:3 %only plot z axis
    figure(i); clf;
    plot(timeStepsTrimmed,xTrimmed(i+1,:),'-.ob');
    hold on; grid on;
    xlabel('time [s]');
    if(i == 3) %for z axis
      plot(timeStepsTrimmed(1,[1,end]),[touchPoint,touchPoint],'g','LineWidth',2)
      plot(timeStepsTrimmed(1,[1,end]),[staticSpringStretchingPoint,staticSpringStretchingPoint],'y','LineWidth',2)
    end
    
    ylabel(yLabels{i}); title(titleLabels{i})
    options.Format = 'eps';
    hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',titleLabels{i},'.eps'],options);
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
figure(83); clf;
plot(dt1,'r.')
hold on
grid on
plot(dt,'b.')
title('different dt')

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

legend('mode','thetad = x(5,:) ','thetad = xd(2,:)','gradient','diff')
plotName = 'Velocities thetad =  - Different Methods';
title(plotName)
axis([84,105,-inf,inf]);
options.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',plotName,'.eps'],options);

for i=1:3
    
    figure(85+i); clf; grid on;
    plot((xTrimmed(1,:)),'m-..')
    hold on
    grid on
    plot(xdTrimmed(4+i,:),'k-.^')
    plot(xdTrimmedGrad(4+i,:),'b-.o')
    plot(xdTrimmedDiff(4+i,:),'g-.+')
    legend('mode','xd','gradient','diff')
    plotName = ['Accelerations ','xd(',num2str(4+i),')',' Comparision- Different Methods'];
    title(plotName)
    axis([84,105,-inf,inf]);
    
    options.Format = 'eps';
    hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',plotName,'.eps'],options);
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
    figure(21); hold on; grid on;
    plot(timeSteps,xTrimmed(4,:),'g','LineWidth',1.0);
    plot(timeStepsSplitNC,xTrimmedNC(4,:),'b.','LineWidth',2.0)
    plot(timeSteps([1,end]),[zTouch,zTouch],'g','LineWidth',2)
    plot(timeStepsSplitIC,xTrimmedIC(4,:),'r.','LineWidth',2.0)
    xlabel('time [s]')
    %axis([-inf inf minHeight maxHeight])
    plotName = ['Height z - Unseparated (',dataSetName,')'];
    title(plotName);
    typeofPlot = 'z';
    options.Format = 'eps';
    hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
end

%%
if(generatePlot)
    
    figure(22); clf; hold on; grid on;
    plot(timeStepsSplitNC,xTrimmedNC(4,:),'g.','LineWidth',3.0)
    plot(timeSteps(1,[1,end]),[zTouch,zTouch],'g','LineWidth',1.5)
    plot(timeStepsSplitIC,xTrimmedIC(4,:),'r.','LineWidth',3.0)
    %axis([-inf inf minHeight maxHeight])
    xlabel('time $t [s]$','Interpreter','LaTex')
    ylabel('$z$','Interpreter','LaTex')
    title(['Height $z$ - Separated (',dataSetName,')'],'Interpreter','LaTex')
    
    figure(23);clf; hold on; grid on;
    xlabel('time $t [s]$','Interpreter','LaTex')
    ylabel('$\dot{z}$','Interpreter','LaTex')
    title(['Velocity $\dot{z}$',' (',dataSetName,')'],'Interpreter','LaTex')
    
    figure(24);clf; hold on; grid on;
    xlabel('time $t [s]$','Interpreter','LaTex')
    ylabel('$\ddot{z}$','Interpreter','LaTex')
    title(['Acceleration $\ddot{z}$',' (',dataSetName,')'],'Interpreter','LaTex')
    
    figure(32); clf; hold on; grid on;
    plot(timeStepsSplitNC,xTrimmedNC(2,:),'g.','LineWidth',3.0)
    plot(timeStepsSplitIC,xTrimmedIC(2,:),'r.','LineWidth',3.0)
    %axis([-inf inf minHeight maxHeight])
    xlabel('time $t [s]$','Interpreter','LaTex')
    ylabel('$\theta$','Interpreter','LaTex')
    title(['Angle $\theta$ (',dataSetName,')'],'Interpreter','LaTex')
    
    figure(33);clf; hold on; grid on;
    xlabel('time $t [s]$','Interpreter','LaTex')
    ylabel('$\dot{\theta}$','Interpreter','LaTex')
    title(['Angular Velocity $\dot{\theta}$',' (',dataSetName,')'],'Interpreter','LaTex')
    
    figure(34);clf; hold on; grid on;
    xlabel('time $t [s]$','Interpreter','LaTex')
    ylabel('$\ddot{\theta}$','Interpreter','LaTex')
    title(['Angular Acceleration $\ddot{\theta}$',' (',dataSetName,')'],'Interpreter','LaTex')
    
    figure(42); clf; hold on; grid on;
    plot(timeStepsSplitNC,xTrimmedNC(3,:),'g.','LineWidth',3.0)
    plot(timeStepsSplitIC,xTrimmedIC(3,:),'r.','LineWidth',3.0)
    %axis([-inf inf minHeight maxHeight])
    xlabel('time [s]')
    ylabel('$x$','Interpreter','LaTex')
    title(['Height $x$ - Separated (',dataSetName,')'],'Interpreter','LaTex')
    
    figure(43);clf; hold on; grid on;
    xlabel('time $t [s]$','Interpreter','LaTex')
    ylabel('$\dot{x}$','Interpreter','LaTex')
    title(['Velocity $\dot{x}$',' (',dataSetName,')'],'Interpreter','LaTex')
    
    figure(44);clf; hold on; grid on;
    xlabel('time $t [s]$','Interpreter','LaTex')
    ylabel('$\ddot{x}$','Interpreter','LaTex')
    title(['Acceleration $\ddot{x}$',' (',dataSetName,')'],'Interpreter','LaTex')
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


if(generatePlot)
  
  figure(22);
  typeofPlot = 'zsep';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(23);
  typeofPlot = 'zd';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(24);
  typeofPlot = 'zdd';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(32);
  typeofPlot = 'theta';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(33);
  typeofPlot = 'thetad';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(34);
  typeofPlot = 'thetadd';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(42);
  typeofPlot = 'x';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(43);
  typeofPlot = 'xd';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(44);
  typeofPlot = 'xdd';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
end



save(newFilename,'angleDeg','mdisc','spread','zTouch',...
  'timeStepsNC','thetaNC','thetadNC','thetaddNC','xNC','xdNC','xddNC','zNC','zdNC','zddNC',...
  'timeStepsIC','thetaIC','thetadIC','thetaddIC','xIC','xdIC','xddIC','zIC','zdIC','zddIC','paramsUsed');

