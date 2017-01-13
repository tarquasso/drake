%% Find the fixed point of the Poincare return map

% Initialize the plant
p = ChineseYoYo();

% Initial Guess for the Fixed Point
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = 0;
x0.load_z = 0.4672;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Construct a visualizer
v = p.constructVisualizer();

% Define a tolerance and a maximum number of iterations for the algorithm
tol = 1e-6;
nMax = 25;

a = -0.05;
x0(3) = a;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Simulate until the next apex
tic; [ytraj, xtraj] = simulate(p, [0, 1], x0); toc
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));

% Find the index at which the ball reaches its apex
[apex, ind] = max( yAll(4, jumpIdx(2):jumpIdx(3)) );

xAtApex = yAll(3, jumpIdx(2) + ind - 1);
fa = x0(3) - xAtApex;


b = 0.05;
x0(3) = b;
x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

% Simulate until the next apex
tic; [ytraj, xtraj] = simulate(p, [0, 1], x0); toc
tt = getBreaks(ytraj);
yAll = ytraj.eval(tt);

% Find jumping indices
jumpIdx = find(diff(yAll(1,:)));

% Find the index at which the ball reaches its apex
[apex, ind] = max( yAll(4, jumpIdx(2):jumpIdx(3)) );

xAtApex = yAll(3, jumpIdx(2) + ind - 1);
fb = x0(3) - xAtApex;

n = 1;
fprintf(['Starting the iteration: n = ', num2str(n), '\n']);
while n <= nMax
  c = 1/2*(a+b);

  x0(3) = c;
  x0(2:end) = resolveConstraints(p.no_contact, x0(2:end));

  % Simulate until the next apex
  [ytraj, xtraj] = simulate(p, [0, 1], x0);
  tt = getBreaks(ytraj);
  yAll = ytraj.eval(tt);

  % Find jumping indices
  jumpIdx = find(diff(yAll(1,:)));

  % Find the index at which the ball reaches its apex
  [apex, ind] = max( yAll(4, jumpIdx(2):jumpIdx(3)) );

  xAtApex = yAll(3, jumpIdx(2) + ind - 1);
  fc = x0(3) - xAtApex;

  if fc == 0 || 1/2*(b-a) < tol
    return
  end
  n = n + 1;
  fprintf(['n = ', num2str(n)]);
  fprintf([', f(c) = ', num2str(fc), '\n']);

  if sign(fc) == sign(fa)
    a = c;
  else
    b = c;
  end
end
