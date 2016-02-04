function paramEstSyntheticDataURDF
tStart0 = tic;
tmp = addpathTemporary(fullfile(pwd,'..'));

%% CONFIGURATION
% Introduce parameter error into estimation
hasParamErr = true;
% Standard deviation of the parameter percent error
paramstd = .20;

% Introduce measurement noise into estimation
hasMeasNoise = true;
% Standard deviation of error for theta1,theta2,theta1dot,theta2dot,theta1doubledot,theta2doubledot
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
simMethod = 'euler';

% Method by which to obtain qdd (only used in dynamic model)
% 'manipul'     = Use acrobot manipulator equations to estimate true qdd
% 'derivative'  = Take the derivative of qd
qddmode = 'derivative';

% The symmetric positive definite cost matrix for simerr
% Err=sum((xobs(n)-xhat(n))'*C*(xobs(n)-xhat(n)))
parameterEstimationOptions.C = eye(4);

%% Initialize Acrobot plants and variables
rtrue = RigidBodyManipulator('AcrobotWSixParams.urdf');
r = rtrue;
nq = r.getNumPositions;
outputFrameNames = r.getOutputFrame.getCoordinateNames();
p_orig = double(r.getParams);
np = length(p_orig);
pnames = getCoordinateNames(getParamFrame(r));

%% Initialize estimated Acrobot system with parameter error
if hasParamErr
  % Perturb original parameter estimates with random percentage error
  % normally distributed with standard dev = paramstd, and greater than -1
  rndVals = randn(1,np);
  paramerr = rndVals*paramstd;
  
  while sum(paramerr<=-1)~=0
    paramerr(paramerr<-1) = randn(1,sum(paramerr<-1))*paramstd;
  end
  
  pErr = p_orig; %first copy true values to then disturb them
  for i = 1:np
    pErr(i) = pErr(i)  + pErr(i) * paramerr(i);
  end
  
  r = r.setParams(pErr); %update the parameters and then copy it over (since no pass by reference)
end

%% Generate swingup data
fprintf('Generating Swing Up Trajectory...\n');
[utraj,xtraj] = swingUpURDF(rtrue); % Output is "Elapsed time is ... seconds"
breaks=getBreaks(utraj); T0 = breaks(1); Tf = breaks(end);
Ts = 0.0001; %10 * (Tf-T0)/(numel(breaks)-1);
tsamples = T0:Ts:Tf;
usamples = eval(utraj,tsamples)';
if strcmp(simMethod,'dircol')
    fprintf('Computing Trajectory using Direct Collocation...\n');
    tStart1 = tic;
    xsamples = eval(xtraj,tsamples)';
    toc(tStart1)
elseif strcmp(simMethod,'euler')
    fprintf('Computing Trajectory using Forward Euler Method...\n');
    tStart2 = tic;
    xsamples = computeTraj(rtrue,eval(xtraj,T0),usamples',tsamples)'; %depending on step size, this might take some time to compute
    toc(tStart2)
else error('Must choose a simulation method'); end

%% Add gaussian noise to measurements
if hasMeasNoise
    measurementNoise = randn(size(xsamples))*diag(noisestd(1:size(xsamples,2)));
else
    measurementNoise = 0;
end
xsamplesnoisy = xsamples+measurementNoise;

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
    usamples = usamples(1:length(qdd),:);
    outputFrameNames = [outputFrameNames;'theta1doubledot';'theta2doubledot'];
else
    xsamplesfinal = xsamplesnoisy;
end

fprintf('Perform Parameter Estimation ...\n');
tStart3 = tic;
data = iddata(xsamplesfinal,usamples,Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',outputFrameNames);
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

function xtraj = computeTraj(obj,x0,utraj,t)
    nx = length(x0);
    dt = diff(t);
    N = length(dt);
    xtraj = [x0,zeros(nx,N)];
    for i=1:N
        f = dynamics(obj,t(i),xtraj(:,i),utraj(:,i));
        xtraj(:,i+1) = xtraj(:,i)+f*dt(i);
    end
end
