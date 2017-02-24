%% Initializations
% Build Up the Plant
p = SoftPaddleHybridReal();
plantSim = SimulinkModel(p.getModel());

% Take the frames from the Simulink model and use those for the simulation of
% the plant and the controller
p = setOutputFrame(p, getOutputFrame(plantSim));
p = setInputFrame(p, getInputFrame(plantSim));
c = SoftPaddlePositionControllerReal(p);

% Desired paddle angle
c.psiDes = 0;

% Interconnect the plant and the controller
output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();
sys = mimoFeedback(plantSim, c, [], [], [], output_select);

% Amount of perturbation along the (x,z) and xdot directions
dPos = 1e-05;
dVel = 5e-04;
% dAngle = 1e-04;

% Initial State of the system:
% Start at the desired apex value with zero velocity
xNominal = -0.00129928588867188;
zNominal = 0.4672;
xpNominal = 0;
psiNominal = 0;

x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = xNominal;
x0.load_z = zNominal;
x0.load_xdot = xpNominal;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Initialize the state and input matrices
A = NaN*ones(3,3);
B = NaN*ones(3,3);

% Construct a visualizer for the system
v = p.constructVisualizer();
v.drawWrapper(0, x0);

%% Perturbations and evaluations
% Perturb along the x-direction

% Negative perturbation
x0(4) = xNominal - 1/2*dPos;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Simulate until the next apex
[ytraj, xtraj] = simulate(sys, [0, 1], x0);
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));
xnplus1 = yAll([4,5,8],jumpIdx(3));


% Positive perturbation
x0(4) = xNominal + 1/2*dPos;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Simulate until the next apex
[ytraj, xtraj] = simulate(sys, [0, 1], x0);
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));
xnplus2 = yAll([4,5,8],jumpIdx(3));

A(:,1) = 1/dPos*(xnplus2 - xnplus1)
x0(4) = xNominal;

% Perturb along the z-direction

% Negative perturbation
x0(5) = zNominal - 1/2*dPos;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Simulate until the next apex
[ytraj, xtraj] = simulate(sys, [0, 1], x0);
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));
xnplus1 = yAll([4,5,8],jumpIdx(3));


% Positive perturbation
x0(5) = zNominal + 1/2*dPos;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Simulate until the next apex
[ytraj, xtraj] = simulate(sys, [0, 1], x0);
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));
xnplus2 = yAll([4,5,8],jumpIdx(3));

A(:,2) = 1/dPos*(xnplus2 - xnplus1)
x0(5) = zNominal;


% Perturb along the xdot-direction

% Negative perturbation
x0(8) = xpNominal - 1/2*dVel;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Simulate until the next apex
[ytraj, xtraj] = simulate(sys, [0, 1], x0);
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));
xnplus1 = yAll([4,5,8],jumpIdx(3));


% Positive perturbation
x0(8) = xpNominal + 1/2*dVel;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Simulate until the next apex
[ytraj, xtraj] = simulate(sys, [0, 1], x0);
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));
xnplus2 = yAll([4,5,8],jumpIdx(3));

A(:,3) = 1/dVel*(xnplus2 - xnplus1)
x0(8) = xpNominal;



% Perturb along the psi-direction

% Negative perturbation
x0(2) = psiNominal - 1/2*dPos;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

c.psiDes = psiNominal - 1/2*dPos;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

% Simulate until the next apex
[ytraj, xtraj] = simulate(sys, [0, 1], x0);
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));
xnplus1 = yAll([4,5,8],jumpIdx(3));


% Positive perturbation
x0(2) = psiNominal + 1/2*dPos;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

c.psiDes = psiNominal + 1/2*dPos;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

% Simulate until the next apex
[ytraj, xtraj] = simulate(sys, [0, 1], x0);
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));
xnplus2 = yAll([4,5,8],jumpIdx(3));

B = 1/dPos*(xnplus2 - xnplus1)
x0(2) = psiNominal;



% % Perturb along the gamma1-direction
%
% % Negative perturbation
% x0(2) = psiNominal;
% x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));
%
% c.gamma1 = psiNominal - 1/2*dPos;
%
% output_select(1).system = 1;
% output_select(1).output = plantSim.getOutputFrame();
% output_select(2).system = 2;
% output_select(2).output = c.getOutputFrame();
%
% sys = mimoFeedback(plantSim,c,[],[],[],output_select);
%
% % Simulate until the next apex
% [ytraj, xtraj] = simulate(sys, [0, 1], x0);
% tt = getBreaks(ytraj);
% yAll = ytraj.eval(tt);
%
% % Find jumping indices
% jumpIdx = find(diff(yAll(1,:)));
% xnplus1 = yAll([4,5,8],jumpIdx(3));
%
%
% % Positive perturbation
% x0(2) = psiNominal;
% x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));
%
% c.gamma1 = psiNominal + 1/2*dPos;
%
% output_select(1).system = 1;
% output_select(1).output = plantSim.getOutputFrame();
% output_select(2).system = 2;
% output_select(2).output = c.getOutputFrame();
%
% sys = mimoFeedback(plantSim,c,[],[],[],output_select);
%
% % Simulate until the next apex
% [ytraj, xtraj] = simulate(sys, [0, 1], x0);
% tt = getBreaks(ytraj);
% yAll = ytraj.eval(tt);
%
% % Find jumping indices
% jumpIdx = find(diff(yAll(1,:)));
% xnplus2 = yAll([4,5,8],jumpIdx(3));
%
% B(:,1) = 1/dPos*(xnplus2 - xnplus1)
%
%
%
% % Perturb along the gamma2-direction
%
% % Negative perturbation
% x0(2) = psiNominal;
% x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));
%
% c.gamma2 = psiNominal - 1/2*dPos;
%
% output_select(1).system = 1;
% output_select(1).output = plantSim.getOutputFrame();
% output_select(2).system = 2;
% output_select(2).output = c.getOutputFrame();
%
% sys = mimoFeedback(plantSim,c,[],[],[],output_select);
%
% % Simulate until the next apex
% [ytraj, xtraj] = simulate(sys, [0, 1], x0);
% tt = getBreaks(ytraj);
% yAll = ytraj.eval(tt);
%
% % Find jumping indices
% jumpIdx = find(diff(yAll(1,:)));
% xnplus1 = yAll([4,5,8],jumpIdx(3));
%
%
% % Positive perturbation
% x0(2) = psiNominal;
% x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));
%
% c.gamma2 = psiNominal + 1/2*dPos;
%
% output_select(1).system = 1;
% output_select(1).output = plantSim.getOutputFrame();
% output_select(2).system = 2;
% output_select(2).output = c.getOutputFrame();
%
% sys = mimoFeedback(plantSim,c,[],[],[],output_select);
%
% % Simulate until the next apex
% [ytraj, xtraj] = simulate(sys, [0, 1], x0);
% tt = getBreaks(ytraj);
% yAll = ytraj.eval(tt);
%
% % Find jumping indices
% jumpIdx = find(diff(yAll(1,:)));
% xnplus2 = yAll([4,5,8],jumpIdx(3));
%
% B(:,2) = 1/dPos*(xnplus2 - xnplus1)
%
%
%
% % Perturb along the gamma3-direction
%
% % Negative perturbation
% x0(2) = psiNominal;
% x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));
%
% c.gamma3 = psiNominal - 1/2*dPos;
%
% output_select(1).system = 1;
% output_select(1).output = plantSim.getOutputFrame();
% output_select(2).system = 2;
% output_select(2).output = c.getOutputFrame();
%
% sys = mimoFeedback(plantSim,c,[],[],[],output_select);
%
% % Simulate until the next apex
% [ytraj, xtraj] = simulate(sys, [0, 1], x0);
% tt = getBreaks(ytraj);
% yAll = ytraj.eval(tt);
%
% % Find jumping indices
% jumpIdx = find(diff(yAll(1,:)));
% xnplus1 = yAll([4,5,8],jumpIdx(3));
%
%
% % Positive perturbation
% x0(2) = psiNominal;
% x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));
%
% c.gamma3 = psiNominal + 1/2*dPos;
%
% output_select(1).system = 1;
% output_select(1).output = plantSim.getOutputFrame();
% output_select(2).system = 2;
% output_select(2).output = c.getOutputFrame();
%
% sys = mimoFeedback(plantSim,c,[],[],[],output_select);
%
% % Simulate until the next apex
% [ytraj, xtraj] = simulate(sys, [0, 1], x0);
% tt = getBreaks(ytraj);
% yAll = ytraj.eval(tt);
%
% % Find jumping indices
% jumpIdx = find(diff(yAll(1,:)));
% xnplus2 = yAll([4,5,8],jumpIdx(3));
%
% B(:,3) = 1/dPos*(xnplus2 - xnplus1)








%% Calculate Discrete Linear Quadratic Regulator
Q = diag([10,10,1]);
R = 1e-03*eye(1);
[K,S,E] = dlqr(A,B,Q,R);
[V,D] = eig(A-B*K);

Ts = 1;
sys = ss(A-B*K, B, eye(3), zeros(3,1), Ts);
t = 0:Ts:10;
u = zeros(1, length(t));
% x0 = [1.7969; -0.31027; -9.8323];
% x0 = [-1.781; -7.1151; 6.7973];
x0 = [0.069302; -9.9956; -0.29013];


%% Plot the linear system characteristics
figure(4), clf, hold on
h = ezplot('x^2 + y^2 = 1');
set(h, 'Color', [0,0,0], 'LineWidth', 2);
plot(real(diag(D)), imag(diag(D)), 'x', 'MarkerSize', 10)
axis([-1,1,-1,1])


figure(5), clf
lsim(sys, u, t, x0);
