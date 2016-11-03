function tensionParamEstSyntheticDataURDF
tStart0 = tic;
tmp = addpathTemporary(fullfile(pwd,'..'));

%% CONFIGURATION
timeToSim = 1; %seconds
generateNewData = true;

%Sampling Time
Ts = 0.01; %this tremedously changes simulation time

% Introduce parameter error into estimation
hasParamErr = true;
% Standard deviation of the parameter percent error
paramstd = .20;

% Introduce measurement noise into estimation
hasMeasNoise = false;
% Standard deviation of error for q1,q2,q1dot,q2dot,q1doubledot,q2doubledot
% noisestd = sqrt([.000001, .000001, .0000001, .0000001, 0, 0,]);
noisestd = sqrt([1e-4, 1e-4, 1e-11, 1e-11, 0, 0,]);

% Introduce delay into estimation (Not complete)
delay = false;

% Parameter Estimation model
% 'dynamic'     = use dynamic model - requires qdd
% 'energetic'   = use energetic model - doesn't require qdd
% 'simerr'      = use simulation error - doesn't require qdd
parameterEstimationOptions.model = 'dynamic';

% Parameter Estimation method
% 'nonlinprog'  = nonlinear least squares (LS) to solve problem
% 'linprog'       = linear LS on lumped params then nonlinear LS to recover
%                 original parameters
% 'lsqnonlin'   = use MATLAB's built-in nonlinear least squares solver (debugging)
parameterEstimationOptions.method = 'nonlinprog';

% Option to print from estimator
% 'noprint'     = Do not print output from parameterEstimation.m, but will
%                 still print output of paramEstSyntheticData.m script
% 'printEst'    = only print estimated from parameterEstimation.m
% 'printAll'	= print estimated and original from parameterEstimation.m
parameterEstimationOptions.print_result = 'noprint';

% Forward simulation model to produce ground truth state trajectory
% 'dircol'      = use direct collocation method (will not work with simerr)
% 'euler'       = use forward Euler method
simMethod = 'dircol';

% Method by which to obtain qdd (only used in dynamic model)
% 'manipul'     = Use acrobot manipulator equations to estimate true qdd
% 'derivative'  = Take the derivative of qd
qddmode = 'derivative';

% The symmetric positive definite cost matrix for simerr
% Err=sum((xobs(n)-xhat(n))'*C*(xobs(n)-xhat(n)))
parameterEstimationOptions.C = eye(4);

%% Initialize tension plants and variables
% TODO: find out if just a PlanarRigidBodyManipulator would also work??
%rtrue =  TimeSteppingRigidBodyManipulator('tensionWParams.urdf',Ts,struct('twoD',true));  
rtrue = PlanarRigidBodyManipulator('tensionWParams.urdf');
r = rtrue;
nq = r.getNumPositions;
outputFrameNamesAll = r.getOutputFrame.getCoordinateNames();
outputFrameNamesDisk = outputFrameNamesAll([2 3 5 6]);
p_orig = double(r.getParams);
np = length(p_orig);
%pnames = getCoordinateNames(getParamFrame(r));

%get a handle on the manipulator plant
if(isa(rtrue,'TimeSteppingRigidBodyManipulator'))
  manip = rtrue.getManipulator;
elseif(isa(rtrue,'PlanarRigidBodyManipulator'))
  manip = rtrue;
else
  error('not supported type of plant');
end
  
%Check if plant is position constrained
if(~isempty(manip.position_constraints))
  isPositionConstrained = true;
  nc = numel(manip.position_constraints);
else
  isPositionConstrained = false;
  nc = 0;
end

% Set Output Frame to what is observable
% stateFrame = getStateFrame(r);
% constructFrame
% r.setOutputFrame(stateFrame);


%% Initialize estimated tension system with parameter error
if hasParamErr
  % Perturb original parameter estimates with random percentage error
  % normally distributed with standard dev = paramstd, and greater than -1
  rndVals = randn(1,np);
  errorFactor = rndVals*paramstd;
  
  %get parameter bounds of the parsed in tension plant
  [pMin,pMax] = getParamLimits(r);
  
  pErr = p_orig; %first copy true values to then disturb them
  for i = 1:np
    pErr(i) = pErr(i)  + pErr(i) * errorFactor(i);
    
    %Check if within parameter bounds
    if(pErr(i) < pMin(i))
      %set it to lower bound
      pErr(i) = pMin(i);
    elseif(pErr(i)> pMax(i))
      %set it to upper bound
      pErr(i) = pMax(i);
    end
    
  end
  r = r.setParams(pErr); %update the parameters and then copy it over (since no pass by reference)
end

%% Generate disk drop data
%check if previously recorded data is available
testingDataExists = exist('testingData.mat','file');
if(testingDataExists ~= 2 || generateNewData) %only generate if no previous data is there

fprintf('Generating Disk Drop on Tensioner Trajectory...\n');
xtraj = diskOnTensionerDrop(rtrue,timeToSim); % Output is "Elapsed time is ... seconds"
breaks=getBreaks(xtraj); T0 = breaks(1); Tf = breaks(end);
%Ts = 10 * (Tf-T0)/(numel(breaks)-1);
tsamples = breaks; %T0:Ts:Tf;
if strcmp(simMethod,'dircol')
  fprintf('Computing Trajectory using Direct Collocation...\n');
  tStart1 = tic;
  xSamplesAll = eval(xtraj,tsamples)';
  toc(tStart1)
elseif strcmp(simMethod,'euler')
  fprintf('Computing Trajectory using Forward Euler Method...\n');
  tStart2 = tic;
  xSamplesAll = computeTraj(manip,eval(xtraj,T0),tsamples)'; %depending on step size, this might take some time to compute
  toc(tStart2)
else error('Must choose a simulation method'); end

%% Remove Unobservable components
% TODO shouldn't that be something that can be done in the simulate
% function already?
xsamplesDisk = xSamplesAll(:,[2:(nq),(nq+2):(2*nq)]); %TODO: remove the theta_angle and its derivative
nq = nq-nc; %TODO: do this more elegant adjust dimension
 
%% Add gaussian noise to measurements
if hasMeasNoise
  measurementNoise = randn(size(xsamplesDisk))*diag(noisestd(1:size(xsamplesDisk,2)));
else
  measurementNoise = 0;
end
xsamplesnoisy = xsamplesDisk+measurementNoise;

%% Generate second derivative
if strcmp(parameterEstimationOptions.model,'dynamic')
  if strcmp(qddmode,'manipul')
    qdd = zeros(length(tsamples),nq);
    for j=1:length(tsamples)
      [H,C,B] = manipulatorDynamics(rtrue,xsamplesnoisy(j,1:nq)',xsamplesnoisy(j,nq+(1:nq))');
      qdd(j,:) = (H\(B*usamples(j,:)' - C))';
    end
  elseif strcmp(qddmode,'derivative')
    % Differentiating to get the second derivative of the state variables
    % TODO: Try lowpass filter
    dtsamples = diff(tsamples)';
    qdd = diff(xsamplesnoisy(:,nq+(1:nq)),1,1);
    qdd = qdd./repmat(dtsamples,1,size(qdd,2));
  else error('Must choose a qddmode'); end
  
  if hasMeasNoise
    qdd = qdd + randn(size(qdd))*diag(noisestd((end+1-size(qdd,2)):end));
  end
  xsamplesfinal = [xsamplesnoisy(1:length(qdd),:), qdd];
  %usamples = usamples(1:length(qdd),:);
  outputFrameNamesDisk = [outputFrameNamesDisk;'load_xdotdot';'load_zdotdot'];
else
  xsamplesfinal = xsamplesnoisy;
end

fprintf('Perform Parameter Estimation ...\n');
tStart3 = tic;
data = iddata(xsamplesfinal,[],Ts,'OutputName',outputFrameNamesDisk); %'InputName',r.getInputFrame.getCoordinateNames(),

% now that it generated new data, save it
  save('testingData.mat','data');
else
  load('testingData.mat');
end
[estimated_parameters,simerror] = parameterEstimation(r,data,parameterEstimationOptions);
toc(tStart3)

%% Print out results
coords = getCoordinateNames(r.getParamFrame);
p_true = double(rtrue.getParams);
p_init = double(r.getParams);
fprintf('\nParameter estimation results:\n\n');
fprintf('  Param  \tTrue    \t Initial\tEstimated\n');
fprintf('  -----  \t--------\t--------\t---------\n');
for i=1:length(coords)
  fprintf('%7s  \t%8.2f\t%8.2f\t%8.2f\n',coords{i},p_true(i),p_init(i),estimated_parameters(i));
end
fprintf('Simulation Error: %s:\n',simerror);
totalTime = toc(tStart0);
fprintf('Total Execution Time: %9.6f seconds:\n',totalTime);

end

function xtraj = computeTraj(obj,x0,t)
nx = length(x0);
dt = diff(t);
N = length(dt);
xtraj = [x0,zeros(nx,N)];
for i=1:N
  f = dynamics(obj,t(i),xtraj(:,i)); %TODO Speed this up!
  xtraj(:,i+1) = xtraj(:,i)+f*dt(i);
end
end

function xtraj = diskOnTensionerDrop(obj,timeToSim)

x0 = Point(getStateFrame(obj));
x0.load_x = 0;
x0.load_z = 3.99;
x0.load_zdot = -2.5; %starting velocity
x0 = resolveConstraints(obj,x0);

xtraj = simulate(obj,[0 timeToSim],x0);

end

