q = rand(10,3);
qd = 10*rand(10,3);
qdd = 100*rand(100,3);

p0 = rand(4,1); % simulation hand tuning needed for this parameter

fun = @(p) paramEstCostFun(p, q, qd, qdd);
options = optimoptions(@fmincon);
options = optimoptions(options, 'Display', 'iter');

[x,fval] = fmincon(fun, p0, [],[], [], [], 0.01*ones(4,1), 1000*ones(4,1),[], options);