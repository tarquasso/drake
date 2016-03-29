function crazyflie_param_est
tStart0 = tic;
tmp = addpathTemporary(fullfile(pwd,'..'));

%% CONFIGURATION

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

qddmode = 'derivative';

parameterEstimationOptions.C = eye(12);

%% Initialize plants and variables
options.floating = true;
cf = RigidBodyManipulator('crazyflieWParams.urdf',options);
r = cf;
nq = r.getNumPositions;
outputFrameNames = r.getOutputFrame.getCoordinateNames();
p_orig = double(r.getParams);
np = length(p_orig);
pnames = getCoordinateNames(getParamFrame(r));

%% Generate derivative
load('sysidData.mat');
data = getexp(z,'Exp5');

Ts = get(data,'Ts');
tsamples = get(data,'SamplingInstants');
x_data = get(data,'OutputData')';
q = x_data(1:nq,:)';
usamples = get(data,'InputData');

% Differentiating to get the second derivative of the state variables
% TODO: Try lowpass filter
dtsamples = diff(tsamples);
qd = diff(q,1,1);
qd = qd./repmat(dtsamples,1,size(qd,2));

xsamples = [q(1:length(qd),:), qd];
usamples = usamples(1:length(qd),:);
tsamples = tsamples(1:length(qd));

%% Generate second derivative
if strcmp(parameterEstimationOptions.model,'dynamic')
  % Differentiating to get the second derivative of the state variables
  % TODO: Try lowpass filter
  dtsamples = diff(tsamples);
  qdd = diff(xsamples(:,nq+(1:nq)),1,1);
  qdd = qdd./repmat(dtsamples,1,size(qdd,2));
  xsamplesfinal = [xsamples(1:length(qdd),:), qdd];
  usamples = usamples(1:length(qdd),:);
  outputFrameNames = [outputFrameNames;'base_xddot';'base_yddot';'base_zddot';'base_rollddot';'base_pitchddot';'base_yawddot'];
else
    xsamplesfinal = xsamples;
end

data = iddata(xsamplesfinal,usamples,Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',outputFrameNames);

fprintf('Perform Parameter Estimation ...\n');
tStart3 = tic;
[estimated_parameters,simerror] = parameterEstimation(r,data,parameterEstimationOptions);
toc(tStart3)

%% Print out results
coords = getCoordinateNames(r.getParamFrame);
p_init = double(r.getParams);
fprintf('\nParameter estimation results:\n\n');
fprintf('  Param  \t Initial\tEstimated\n');
fprintf('  -----  \t--------\t---------\n');
for i=1:length(coords)
  fprintf('%7s  \t%8.5e\t%8.5e\n',coords{i},p_init(i),estimated_parameters(i));
end
fprintf('Simulation Error: %s:\n',simerror);
totalTime = toc(tStart0);
fprintf('Total Execution Time: %9.6f seconds:\n',totalTime);

end
