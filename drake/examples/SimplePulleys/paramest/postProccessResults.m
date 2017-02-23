%% Set the following parameters to false after running it for the first time:

%% Parse in Optitrack Capture Data
%loaded preprocessed data set
count = 1;
while( count < 5)
  clearvars -except count
  close all
  if(count == 1)
  dataSetName = 'set10';
  name = 'Juggler-Experiment_2017-01-19_15-59-05';
  elseif(count == 2)
  dataSetName = 'set11';
  name = 'Juggler-Experiment_2017-01-19_16-03-32';
  elseif(count == 3)
  dataSetName = 'set12';
  name = 'Juggler-Experiment_2017-01-19_16-04-44';
  elseif(count == 4)
  dataSetName = 'set13';
  name = 'Juggler-Experiment_2017-01-19_16-05-25';
  end
  
  filename = ['~/Dropbox (MIT)/Robotics Research/soft_modeling_db/Paramater_Estimation_RSS_2017/',dataSetName,'/',name];
  capturedDataFlag = true;

    %   dataSetName = 'set2';
%   name = 'syntheticPaddleDataWithAccel';
%   filename = ['~/soft_modeling_repo/dev/simulation/',dataSetName,'/',name];
%   capturedDataFlag = false;

appendix = '_finalresults';
fullFilenameFinalResults = [filename,appendix,'.mat'];
 [pathstr,name,ext] = fileparts(fullFilenameFinalResults);
 
load(fullFilenameFinalResults)

paramsEstimatedIndividuals
timeStepsNC_max = zeros(size(timeStepsNC));
zNC_max = zeros(size(zNC));
figure
hold on
for k=1:length(zNC)

[zNC_max(k),idx] = max(zNC{k});
timeStepsNC_max(k) = timeStepsNC{k}(idx);
plot(timeStepsNC{k},zNC{k},'k.-.')
end
plot(timeStepsNC_max,zNC_max,'ro')
xlabel('Time $t \ [s]$','Interpreter','LaTex')
ylabel('Height $z \ [m]$','Interpreter','LaTex')
title('Maxima from no contact phases')

g = fittype( @(a,b,c,x) a*exp(-b*x)+c, 'independent', {'x'},...
     'dependent', 'y' );
ft=fittype('exp1');
cf=fit(timeStepsNC_max,zNC_max,g,'StartPoint', [0.68, 0.1,0])

plot(cf)
grid on;
typeofPlot = 'maxima_no_contact';
optionsPlot.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],optionsPlot);
savefig(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.fig']);

% 'angleDeg','mdisc','spread','zTouch',...
%      'paramsEstimated', 'paramsEstimatedIndividuals',...
%       'timeStepsNC','zNC','zdNC','zddNC','zfitNC',...
%       'timeStepsIC','zIC','zdIC','zddIC','zfitIC',...
%       'thetaOrigIC','thetaIC','thetadIC','thetaddIC','thetafitIC');


numOfSets = size(zIC,1);
%rangeOfSets = 1:numOfSets;
%rangeOfSets = 4:numOfSets;
rangeOfSets = 5:min(20,numOfSets);

plotFontSize = 17;
xHandleFontSize = 20;
yHandleFontSize = 20;

figure(1701); clf; hold on;

set(gca,'FontSize',plotFontSize)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSize)
set(yhandle,'Fontsize',yHandleFontSize)

errorbar(zNC_max(rangeOfSets(1:end)),paramsEstimatedIndividuals.I0Est(rangeOfSets),...
    paramsEstimatedIndividuals.I0EstErrLow(rangeOfSets,:),paramsEstimatedIndividuals.I0EstErrUp(rangeOfSets,:),...
'-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','LineWidth',2); %'CapSize',18,
xlabel('Peak Height $z_{max} \ [m]$','Interpreter','LaTex');
ylabel('Inertia $I_s \ [kg m^2]$','Interpreter','LaTex');
%title('Inertia $I_s$ over Peak Height $z_{max}$','Interpreter','LaTex');
% legend(rangeOfModelsCell)
axis([zNC_max(rangeOfSets(end))*0.975 zNC_max(rangeOfSets(1))*1.01 ...
    min(paramsEstimatedIndividuals.I0Est(rangeOfSets)-paramsEstimatedIndividuals.I0EstErrLow(rangeOfSets,:))...
    max(paramsEstimatedIndividuals.I0Est(rangeOfSets)+paramsEstimatedIndividuals.I0EstErrUp(rangeOfSets,:))])
typeofPlot = 'I0Est-height';
%grid on
options.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
savefig(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.fig']);

figure(1702); clf; hold on;

set(gca,'FontSize',plotFontSize)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSize)
set(yhandle,'Fontsize',yHandleFontSize)

errorbar(zNC_max(rangeOfSets(1:end)),paramsEstimatedIndividuals.b0Est(rangeOfSets),...
    paramsEstimatedIndividuals.b0EstErrLow(rangeOfSets,:),paramsEstimatedIndividuals.b0EstErrUp(rangeOfSets,:),...
'-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','LineWidth',2); %'CapSize',18,
xlabel('Peak Height $z_{max} \ [m]$','Interpreter','LaTex');
ylabel('Damping $b \ [\frac{kg m^2}{s}]$','Interpreter','LaTex');
%title('Damping $b$ over Peak Height $z_{max}$','Interpreter','LaTex');
axis([zNC_max(rangeOfSets(end))*0.975 zNC_max(rangeOfSets(1))*1.01  ...
    min(paramsEstimatedIndividuals.b0Est(rangeOfSets)-paramsEstimatedIndividuals.b0EstErrLow(rangeOfSets,:)) ...
    max(paramsEstimatedIndividuals.b0Est(rangeOfSets)+paramsEstimatedIndividuals.b0EstErrUp(rangeOfSets,:))])

% legend(rangeOfModelsCell)
%grid on
typeofPlot = 'b0Est-height';
options.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
savefig(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.fig']);

figure(1703); clf; hold on;

set(gca,'FontSize',plotFontSize)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSize)
set(yhandle,'Fontsize',yHandleFontSize)

errorbar(zNC_max(rangeOfSets),paramsEstimatedIndividuals.k0Est(rangeOfSets),...
    paramsEstimatedIndividuals.k0EstErrLow(rangeOfSets,:),paramsEstimatedIndividuals.k0EstErrUp(rangeOfSets,:),...
'-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','LineWidth',2); %'CapSize',18,
xlabel('Peak Height $z_{max} \ [m]$','Interpreter','LaTex');
ylabel('Stiffness $k \ [\frac{kg m^2}{s^2}]$','Interpreter','LaTex');
%title('Stiffness $k$ over Peak Height $z_{max}$','Interpreter','LaTex');
axis([zNC_max(rangeOfSets(end))*0.975 zNC_max(rangeOfSets(1))*1.01 ...
    min(paramsEstimatedIndividuals.k0Est(rangeOfSets)-paramsEstimatedIndividuals.k0EstErrLow(rangeOfSets,:))...
    max(paramsEstimatedIndividuals.k0Est(rangeOfSets)+paramsEstimatedIndividuals.k0EstErrUp(rangeOfSets,:))])

% legend(rangeOfModelsCell)
%grid on
typeofPlot = 'k0Est-height';
options.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
savefig(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.fig']);


%% B Surface
  bfEstMean = mean(paramsEstimatedIndividuals.bfEst);
  figure(1038);clf;hold on;
  
  set(gca,'FontSize',plotFontSize)
  xhandle=get(gca,'Xlabel');
  yhandle=get(gca,'Ylabel');
  set(xhandle,'Fontsize',xHandleFontSize)
  set(yhandle,'Fontsize',yHandleFontSize)
  
  e = errorbar(zNC_max(rangeOfSets),paramsEstimatedIndividuals.bfEst(rangeOfSets)...
    ,paramsEstimatedIndividuals.bfEstErrLow(rangeOfSets,:),...
    paramsEstimatedIndividuals.bfEstErrUp(rangeOfSets,:),...
    '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','LineWidth',2); %'CapSize',18,
%   e.Marker = '*';
% e.MarkerSize = 10;
% e.Color = 'black';
% e.CapSize = 15;

  hold on;
  
  plot(zNC_max(rangeOfSets([1 end])),[bfEstMean,bfEstMean],'g','LineWidth',1.9);
  
  xlabel('Peak Height $z_{max} \ [m]$','Interpreter','Latex');
  ylabel('Surface Friction $b_f \ [\frac{kg}{s}]$','Interpreter','Latex');
  legend('Individual Estimates','Mean','Location','NorthEast');
  %title('Surface Friction $b_f$ over Peak Height $z_{max}$','Interpreter','Latex');
  %grid on
  axis([zNC_max(rangeOfSets(end))*0.975 zNC_max(rangeOfSets(1))*1.01 ...
    min(paramsEstimatedIndividuals.bfEst(rangeOfSets)-paramsEstimatedIndividuals.bfEstErrLow(rangeOfSets,:))...
    max(paramsEstimatedIndividuals.bfEst(rangeOfSets)+paramsEstimatedIndividuals.bfEstErrUp(rangeOfSets,:))])
  typeofPlot = 'bsurface_errorbar-height';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  savefig(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.fig']);
  

count = count +1;
end
