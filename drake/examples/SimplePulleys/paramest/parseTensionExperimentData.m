function [times, z,zd,zdd, tICE, tICES, xICE, zICE, numOfSets, zfit, gof] = parseTensionExperimentData(filename,touchPoint,discRadius,expStartTime,expEndTime,optiTrackWandErrorFactor,minHeight,maxHeight,generatePlot)

% the following call requires MotionHistory.m to be in the path
load(filename)
History.objectPosition = optiTrackWandErrorFactor* History.objectPosition;

%number of samples
nos = History.i-1;

%%
% all time steps
timeVals = History.timestamps(1:nos); %History.frameTime(1:nos)-History.frameTime(1);

%TODO: Check where y and z values point towards
% x,y,z positions in space
zVals = History.objectPosition(1:nos,1); % z points up on the plane

% adjust in height to the tensionWParamsExp.urdf coordinates
% shift the z coordinate by the disc radius up

zVals = zVals-touchPoint + discRadius;
minHeight = minHeight - touchPoint + discRadius;
maxHeight = maxHeight - touchPoint + discRadius;

xVals = History.objectPosition(1:nos,2);
nVals = History.objectPosition(1:nos,3);

% Shift Frame
zZero = discRadius; 
% finds all values that are beneath the touchpoint
idxInContact = find(zVals < zZero & timeVals > expStartTime & timeVals < expEndTime);

%% Some plotting
if(generatePlot)
  figure(201)
  clf
  plot(timeVals,zVals,'b.-','LineWidth',2)
  hold on
  plot(timeVals(idxInContact),zVals(idxInContact),'r*','LineWidth',1.5)
  plot(timeVals([1,end]),[zZero,zZero],'g','LineWidth',2)
  xlabel('time [s]')
  ylabel('height coordinate z [m]')
  title('Set 5 - Z-Coordinate')
  axis([expStartTime expEndTime minHeight maxHeight])
  options.Format = 'eps';
  hgexport(gcf,sprintf('plots/Set5_all.eps'),options);
  
  h1=gcf;
  h2=figure(202);
  clf
  objects=allchild(h1);
  copyobj(get(h1,'children'),h2);
  
  axis([expStartTime expEndTime minHeight zZero+maxHeight*0.05])
  options.Format = 'eps';
  hgexport(gcf,sprintf('plots/Set5_under.eps'),options);
  
  figure(203);
  clf
  hold on
  plot(timeVals,xVals)
  xlabel('time [s]')
  ylabel('horizontal coordinate [m]')
  title('horizontal coordinate')
  hold on
  axis([expStartTime expEndTime -inf inf])
  
  figure(204);
  clf
  hold on
  
  plot(timeVals,nVals)
  xlabel('time [s]')
  ylabel('normal to plane coordinate [m]')
  title('normal to plane coordinate')
  axis([expStartTime expEndTime -inf inf])
  hold on
  
end

% %% rotate data if necessary
%
% t1_on = find(pos(:,2) > thresh,1);
%
% x = pos_adj(1:t1_on-50,1); %vertical
% y = pos_adj(1:t1_on-50,2); %horizontal
% X = [ones(t1_on-50,1) x];
% b = X\y; %b(2) is slope, which should be 0
% rot = -atan(b(2));
% R = [cos(rot), -sin(rot); sin(rot), cos(rot)];
% pos_rot = R * pos_adj.';
% pos_rot = pos_rot.';
%
% x_adj = pos_rot(:,1); z_adj = pos_rot(:,2);


%% extract all x,t pairsunderneath
zInContact = zVals(idxInContact);
xInContact = xVals(idxInContact);
%nInContact = nVals(idxInContact);

tInContact = timeVals(idxInContact);
% hVec = diff(tUnderLimit);
% Find individual contact phases
newSet = diff(idxInContact)> 1; 

%last step shortens it by one value, so:
startPoints = true(length(newSet)+1,1);
startPoints(2:end) = newSet; %last element is also an ending segment of it
startIdx =  idxInContact(startPoints);

endPoints = true(length(newSet)+1,1);
endPoints(1:end-1) = newSet; %last element is also an ending segment of it
endIdx = idxInContact(endPoints);

%idxExtended = zeros(sum(endIdx-startIdx)+2*length(startIdx),1);
%lastidx = 0;
numOfSets = sum(startPoints);

idxExtended = cell(numOfSets,1);
offsetStep = 1; %defines additional points looked at before or after a contact data set

tICE = cell(numOfSets,1);
tICES = cell(numOfSets,1);
xICE = cell(numOfSets,1);
zICE = cell(numOfSets,1);

for j = 1:numOfSets
  idxExtended{j} = (startIdx(j)-offsetStep):(endIdx(j)+offsetStep);
  tICE{j} = timeVals(idxExtended{j});
  xICE{j} = xVals(idxExtended{j});
  zICE{j} = zVals(idxExtended{j});
  
  if(generatePlot)
    figure(h2)
    plot(tICE{j}(1),zICE{j}(1),'m+','LineWidth',1.5)
    plot(tICE{j}(end),zICE{j}(end),'k*','LineWidth',1.5)
  end
  tICES{j} = tICE{j}-tICE{j}(1);
end

% %% Derivatives 
% % calculate xd values
% xdIC = cell(numOfSets,1);
% zdIC = cell(numOfSets,1);
% for j = 1: numOfSets
%   for i = 1:(length(zIC{j})-1)
%     dt = (tIC{j}(i+1)- tIC{j}(i));
%     xdIC{j}(i+1) = (xIC{j}(i+1) - xIC{j}(i))/dt;
%     zdIC{j}(i+1) = (zIC{j}(i+1) - zIC{j}(i))/dt;    
%   end
% end


%% fit curves
zfit = cell( numOfSets, 1 );
z = cell( numOfSets, 1 );
zd = cell( numOfSets, 1 );
zdd = cell( numOfSets, 1 );

gof = struct( 'sse', cell( numOfSets, 1 ), ...
     'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );
%ft = fittype( 'poly2' );
ft = fittype( 'cubicinterp' );

times = cell( numOfSets, 1 );


elementsToCheck = 10000;
for j = 1: numOfSets
  figure(301); clf; hold on; plot(tICE{j}, zICE{j},'*');
  [tData, zData] = prepareCurveData(  tICE{j}, zICE{j} );
  figure(302); clf; hold on; plot(tData, zData,'*');
  [zfit{j}, gof(j)] = fit( tData, zData, ft );
  
  tEval = linspace(tData(1),tData(2),elementsToCheck);
  fZeroPotentials = feval(zfit{j},tEval);
  [fZeroFirst,idx] = min(abs(fZeroPotentials-zZero));
  tFirst = tEval(idx);
  
  tEval = linspace(tData(end-1),tData(end),elementsToCheck);
  fZeroPotentials = feval(zfit{j},tEval);
  [fZeroLast,idx] = min(abs(fZeroPotentials-zZero));
  tLast = tEval(idx);
%   ub = tICE{j}(1);
%   lb = tICE{j}(2);
%   df = ub-lb;
%   tStart = lb;
%   factor = 0.1;
%   whileflag = true;
%   while(whileflag)
%     
%     [tZero,~,exitflag] = fzero(qfit{j},tStart);
%     
%     if(exitflag<1)
%       tStart = tStart + factor * df;
%     elseif(tZero > ub)
%       %restart with smaller factor
%       tStart = lb;
%       factor = factor*0.1;
%     elseif(tZero < lb)
%       tStart = ub;
%       df = -df;
%     else
%       whileflag = false;
%     end
%     
%   end

times{j} = [tFirst;tICE{j}(2:end-1);tLast];

z{j} = feval(zfit{j},times{j});
[zd{j}, zdd{j}] = differentiate(zfit{j},times{j});

figure(h2)
p = plot(zfit{j},tData,zData);%,[timeInterval{j}])      
p(1).LineWidth = 2;
%p(1).Marker = '--';


end


%% save data
% save(file_name,'t','pos','rads','thresh','contact','pos_adj','pos_rot',...
%     'bounce1','fitresult','bounce1_goodness',...
%     'bounce2','bounce2_fit','bounce2_goodness',...
%     'bounce3','bounce3_fit','bounce3_goodness');

%% Not needed after this
%
% % calculations
% r1 = PlanarRigidBodyManipulator('tensionWParamsExp.urdf'); % Set up rigid body
% nq = r1.num_positions;
% nu = r1.getNumInputs;
%
% %x0 = Point(getStateFrame(r1));
%
% %%
% q=msspoly('q',nq);
% s=msspoly('s',nq);
% c=msspoly('c',nq);
% qt=TrigPoly(q,s,c);
% qd=msspoly('qd',nq);
% qdd=msspoly('qdd',nq);
% u=msspoly('u',nu);
% p=r1.getParamFrame.getPoly;
% pr1 = setParams(r1,p);
% [H,C,B] = manipulatorDynamics(pr1,qt,qd);


%%
%err = H*qdd + C - B*u;
%tau = B*u -C;
%%

%[phi,J] = r1.positionConstraints([0,xuls{1}(2),0]')

% parameters set up

% least squares

end