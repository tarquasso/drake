function [xuls,tuls] = parse_tension_experiment_data(filename,touchPoint,expStartTime,expEndTime)

% the following call requires MotionHistory.m to be in the path
load(filename)

%number of samples
nos = History.i-1;

%%
% all time steps
timeVals = History.timestamps(1:nos); %History.frameTime(1:nos)-History.frameTime(1);

%TODO: Check where y and z values point towards
% x,y,z positions in space
xVals = History.objectPosition(1:nos,1); % x points up on the plane 
yVals = History.objectPosition(1:nos,2); 
zVals = History.objectPosition(1:nos,3);

% finds all values that are beneath the touchpoint
idxUnderLimit = find(xVals<touchPoint & timeVals > expStartTime & timeVals < expEndTime);

%% Some plotting
figure(201)
clf
plot(timeVals,xVals,'b')
hold on
plot(timeVals(idxUnderLimit),xVals(idxUnderLimit),'r*')
plot(timeVals([1,end]),[touchPoint,touchPoint],'g')
xlabel('time [s]')
ylabel('height coordinate z [m]')
title('Set 5 - Z-Coordinate')
axis([5.4 8.2 -0.06 0.93])
options.Format = 'eps';
hgexport(gcf,sprintf('plots/Set5_all.eps'),options);

h1=gcf;
h2=figure(202);
clf
objects=allchild(h1);
copyobj(get(h1,'children'),h2);

axis([5.4 8.2 -0.06 touchPoint+0.01])
options.Format = 'eps';
hgexport(gcf,sprintf('plots/Set5_under.eps'),options);


%% extract all x,t pairsunderneath
xUnderLimit = xVals(idxUnderLimit);
yUnderLimit = yVals(idxUnderLimit);
zUnderLimit = zVals(idxUnderLimit);

tUnderLimit = timeVals(idxUnderLimit);
% hVec = diff(tUnderLimit);
% Find individual contact phases
newSet = diff(idxUnderLimit)> 1; %shortens it by one value

newSet = [newSet;1]; %last element is also an ending segment of it

% total number of contact phases
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
        plot(tUnderLimit(k),xUnderLimit(k),'y*')
        j = j+1;
        i = 0;
    end
        i = i+1;
end

%% 
%results in tULS and xULS cells

% calculate xd values
xduls = cell(numOfSets,1);
for j = 1: numOfSets
for k = 1:(length(xuls{j})-1)

xduls{j}(k+1) = (xuls{j}(k+1) - xuls{j}(k))/(tuls{j}(k+1)- tuls{j}(k));

end
end

%% Fit polynomial


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