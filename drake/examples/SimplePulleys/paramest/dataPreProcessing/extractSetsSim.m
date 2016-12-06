function [numOfSetsNew,timeStepsSplit,xTrimmedSplit] = extractSetsSim(idxMode,timeSteps, xTrimmed,offsetStep,minDataPoints,type)
global generatePlot
%% Define points when in contact and points when not in contact
boolTransitions = diff(idxMode)> 1;
%last step shortens it by one value, so for the start points, add 1 logical in front:
boolTransitionsStarts = [true,boolTransitions];
idxStart =  idxMode(boolTransitionsStarts);
boolTransitionsEnds = [boolTransitions,true];
idxEnd = idxMode(boolTransitionsEnds);

numOfSets = length(idxEnd);%sum(boolTransitionsStarts);

if(strcmp(type,'Split')) % no contact
  
idxStartNew = [];
idxEndNew = [];
for l = 1:numOfSets
  idxCheck = (idxStart(l)):(idxEnd(l));
  zDataRelevant = xTrimmed(idxCheck,4);
  
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
xTrimmedSplit = cell(numOfSetsNew,1);


for j = 1:numOfSetsNew
  idxExtended{j} = (idxStartNew(j)-offsetStep):(idxEndNew(j)+offsetStep);
  timeStepsSplit{j} = timeSteps(idxExtended{j});
  xTrimmedSplit{j} = xTrimmed(:,idxExtended{j});
   
  if(generatePlot)
    figure(22);
    plot(timeStepsSplit{j}(1),xTrimmedSplit{j}(4,1),'m*','LineWidth',4.5)
    plot(timeStepsSplit{j}(end),xTrimmedSplit{j}(4,end),'k*','LineWidth',4.5)
  end
end

end