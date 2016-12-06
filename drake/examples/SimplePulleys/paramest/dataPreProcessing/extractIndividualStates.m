function [timeStepsMode,thetaMode,thetadMode,thetaddMode,...
  xMode,xdMode,xddMode,zMode,zdMode,zddMode] = ...
  extractIndividualStates(numOfSetsMode,timeStepsSplitMode, xTrimmedSplitMode)
global generatePlot

timeStepsMode = cell(numOfSetsMode,1);

dtMode = cell(numOfSetsMode,1);

thetaMode= cell(numOfSetsMode,1);
thetadMode = cell(numOfSetsMode,1);
thetaddMode = cell(numOfSetsMode,1);

xMode= cell(numOfSetsMode,1);
xdMode = cell(numOfSetsMode,1);
xddMode = cell(numOfSetsMode,1);

zMode= cell(numOfSetsMode,1);
zdMode = cell(numOfSetsMode,1);
zddMode = cell(numOfSetsMode,1);

for j = 1: numOfSetsMode
timeStepsMode{j} = timeStepsSplitMode{j}';
dtMode{j} = gradient(timeStepsSplitMode{j}');

thetaMode{j} = xTrimmedSplitMode{j}(2,:)';
thetadMode{j} = xTrimmedSplitMode{j}(5,:)';
%thetaddMode{j} = diff(xTrimmedSplitMode{j}(5,1:end))./dtMode{j};
thetaddMode{j} = gradient(xTrimmedSplitMode{j}(5,1:end)')./dtMode{j};

xMode{j} = xTrimmedSplitMode{j}(3,:)';
xdMode{j} = xTrimmedSplitMode{j}(6,:)';
xddMode{j} = gradient(xTrimmedSplitMode{j}(6,:)')./dtMode{j};

zMode{j} = xTrimmedSplitMode{j}(4,:)';
zdMode{j} = xTrimmedSplitMode{j}(7,:)';
zddMode{j} = gradient(xTrimmedSplitMode{j}(7,:)')./dtMode{j};

if(generatePlot)
    figure(22);
    p1 = plot(timeStepsSplitMode{j}',zMode{j},'-.ob');
    p1(1).LineWidth = 1;
    
    figure(23)
    p2 = plot(timeStepsSplitMode{j}',zdMode{j},'-.ob');
    p2(1).LineWidth = 1;
    
    figure(24)
    p3 = plot(timeStepsSplitMode{j}',zddMode{j},'-.ob');
    p3(1).LineWidth = 1;
  
    figure(32);
    p1 = plot(timeStepsSplitMode{j}',thetaMode{j},'-.ob');
    p1(1).LineWidth = 1;
    
    figure(33)
    p2 = plot(timeStepsSplitMode{j}',thetadMode{j},'-.ob');
    p2(1).LineWidth = 1;
    
    figure(34)
    p3 = plot(timeStepsSplitMode{j}',thetaddMode{j},'-.ob');
    p3(1).LineWidth = 1;
  
end
end
end
