%close all

%% Plotting of ID data and parameter estimation
%Set 5
load('~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_46_41.mat')

optiTrackWandErrorFactor = 1/2;

% Set 5 Touching point: 55.8mm - manually measure from the Optitrack
tp = 0.0558*optiTrackWandErrorFactor;
%Static spring stretching point: 52.8 mm

History.objectPosition = optiTrackWandErrorFactor* History.objectPosition;
%number of samples
nos = History.i-1;

%%
timeVals = History.timestamps(1:nos); %History.frameTime(1:nos)-History.frameTime(1);
xVals = History.objectPosition(1:nos,1);
idxUnderLimit = find(xVals<tp & timeVals > 5 & timeVals < 8);
yVals = History.objectPosition(1:nos,2);
zVals = History.objectPosition(1:nos,3);
figure(201)
clf
plot(timeVals,xVals,'b')
hold on
plot(timeVals(idxUnderLimit),xVals(idxUnderLimit),'r*')
plot(timeVals([1,end]),[tp,tp],'g')
xlabel('time [s]')
ylabel('height coordinate z [m]')
title('Set 5 - Z-Coordinate')
axis([5.4 8.2 -0.03 0.465])
options.Format = 'eps';
hgexport(gcf,sprintf('plots/Set5_all.eps'),options);

h1=gcf;
h2=figure(202);
clf
objects=allchild(h1);
copyobj(get(h1,'children'),h2);

axis([5.4 8.2 -0.06 tp+0.01])
options.Format = 'eps';
title('Set 5 - Z-Coordinate - In contact below Green line')

hgexport(gcf,sprintf('plots/Set5_under.eps'),options);

figure(203);
clf
hold on
plot(timeVals,yVals)
xlabel('time [s]')
ylabel('horizontal coordinate [m]')
title('horizontal coordinate')
hold on
axis([5.4 8.2 -inf inf])

figure(204);
clf
hold on

plot(timeVals,zVals)
xlabel('time [s]')
ylabel('normal to plane coordinate [m]')
title('normal to plane coordinate')
axis([5.4 8.2 -inf inf])
hold on


%% 
% extract more
xUnderLimit = xVals(idxUnderLimit);
tUnderLimit = timeVals(idxUnderLimit);
hVec = diff(tUnderLimit);
% Check for diff larger
newSet = diff(idxUnderLimit)> 1; %shortens it by one value

newSet = [newSet;1]; %last element is also an ending segment of it
numOfSets = sum(newSet);
i = 1;
j = 1;
tuls = cell(numOfSets,1);
xuls = cell(numOfSets,1);
for k= 1:length(xUnderLimit)
    %split into sets
    tuls{j}(i) = tUnderLimit(k);
    xuls{j}(i) = xUnderLimit(k);
    
    if(newSet(k) == 1)
        figure(h2)
        plot(tUnderLimit(k),xUnderLimit(k),'y*')
        j = j+1;
        i = 0;
    end
        i = i+1;
end

%% 
%results in tULS and xULS cells

% more inputs
xduls = cell(numOfSets,1);
for j = 1: numOfSets
for k = 1:(length(xuls{j})-1)

xduls{j}(k+1) = (xuls{j}(k+1) - xuls{j}(k))/(tuls{j}(k+1)- tuls{j}(k));

end
end
%%



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




%%   Step 1: Extract lumped-parameters 

% 
% %%
% x=0.5;
% 
