%build up hybrid plant

p = SoftPaddleHybridReal();
porig = p;
plantSim = SimulinkModel(p.getModel());
% take the frames from the simulink model and use those for the simulation of plant and controller
p = setOutputFrame(p, getOutputFrame(plantSim));
p = setInputFrame(p, getInputFrame(plantSim));

c = SoftPaddlePositionControllerReal_v1(p);

%Adjust the set angle here:
c.psiDes = 0;
c.kappa0 = 0;
c.kappa1 = 0;
c.kappa00 = 0;
c.kappa01 = 0;


output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);
dSpatial = 0.0005;

v = p.constructVisualizer();
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));
v.drawWrapper(0,x0);

%% Perturb along the x direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001-dSpatial/2;
x0.load_z = 0.3;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus1 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus1 = xplus1([3,4,7]);


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001+dSpatial/2;
x0.load_z = 0.3;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));


tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus2 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus2 = xplus2([3,4,7]);

deltax = (xiplus2 -xiplus1)/dSpatial;


%% Perturb along the xdot direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0-dSpatial/2;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus1 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus1 = xplus1([3,4,7]);


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0+dSpatial/2;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus2 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus2 = xplus2([3,4,7]);

deltaxdot = (xiplus2 -xiplus1)/dSpatial;



%% Perturb along the z direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3-dSpatial/2;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus1 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus1 = xplus1([3,4,7]);


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3+dSpatial/2;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus2 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus2 = xplus2([3,4,7]);

deltaz = (xiplus2 -xiplus1)/dSpatial;


%% Perturb along the kappa0 direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

c.kappa0 = 0-dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);


tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus1 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus1 = xplus1([3,4,7]);


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.paddle_angle = 0+dSpatial/2;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

c.kappa0 = 0+dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus2 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus2 = xplus2([3,4,7]);

deltakappa0 = (xiplus2 -xiplus1)/dSpatial;

c.kappa0 = 0;

%% Perturb along the kappa1 direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

c.kappa1 = 0-dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);


tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus1 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus1 = xplus1([3,4,7]);


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.paddle_angle = 0+dSpatial/2;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

c.kappa1 = 0+dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus2 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus2 = xplus2([3,4,7]);

deltakappa1 = (xiplus2 -xiplus1)/dSpatial;

c.kappa1 = 0;

%% Perturb along the kappa0 direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

c.kappa00 = 0-dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);


tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus1 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus1 = xplus1([3,4,7]);


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.paddle_angle = 0+dSpatial/2;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

c.kappa00 = 0+dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus2 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus2 = xplus2([3,4,7]);

deltakappa00 = (xiplus2 -xiplus1)/dSpatial;

c.kappa00 = 0;


%% Perturb along the kappa0 direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

c.kappa01 = 0-dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);


tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus1 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus1 = xplus1([3,4,7]);


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.paddle_angle = 0+dSpatial/2;
x0.load_x = 0.0001;
x0.load_z = 0.3;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

c.kappa01 = 0+dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

tic
[ytraj,xtraj] = simulate(sys,[0 0.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
[zmax, ind] = max(yAll(5,jumpIdx(1):end));
xplus2 = yAll(2:9,jumpIdx(1)+ind-1);
xiplus2 = xplus2([3,4,7]);

deltakappa01 = (xiplus2 -xiplus1)/dSpatial;

c.kappa01 = 0;


%% Calculate system matrices

A = [deltax,deltaz,deltaxdot];
B = [deltakappa0, deltakappa1, deltakappa00, deltakappa01];

%% Calculate DLQR
Q = diag([10,1,1]);
R = 1e-3*eye(4);

[K,S,E] = dlqr(A,B,Q,R);

fprintf(['Gains at xdesired = ', mat2str(round(-K*xi,5)), '\n'])

z = eig(A-B*K);


% save('poincareLinearMirrorLawOutput.mat','A','B','Q','R','K','S','E');

figure(5), clf, hold on
h = ezplot('x^2 + y^2 = 1');
set(h, 'Color', [0 0 0], 'LineWidth', 2)
plot(real(z), imag(z), 'x', 'MarkerSize', 10)
axis([-1,1,-1,1])
axis('square')

Ts = 1;
sys = ss(A-B*K, B, eye(3), zeros(3,4),Ts);
t = 0:Ts:10;
u = zeros(4,length(t));
x0 = [0.1; 0.5; -0.3];

figure(6), clf
lsim(sys, u, t, x0);