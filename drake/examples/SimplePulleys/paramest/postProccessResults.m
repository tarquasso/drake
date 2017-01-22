%% Set the following parameters to false after running it for the first time:

%% Parse in Optitrack Capture Data
%loaded preprocessed data set
count = 1;
while( count < 2)
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
xlabel('time [s]')
ylabel('z [m]')
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


count = count +1;
end
