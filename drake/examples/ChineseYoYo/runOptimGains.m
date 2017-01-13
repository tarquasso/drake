warning off


p = SoftPaddleHybridReal();
porig = p;
plantSim = SimulinkModel(p.getModel());
% take the frames from the simulink model and use those for the simulation of plant and controller
p = setOutputFrame(p, getOutputFrame(plantSim));
p = setInputFrame(p, getInputFrame(plantSim));

tf = 1.5;

fun = @(kappa) myfun(kappa, p, plantSim, tf);
kappa0 = [1e-3; 1e-8; 0.02; 0.015];
A = []; b = [];
Aeq = []; beq = [];
lb = zeros(4,1); ub = ones(4,1);
nonlcon = [];
options = optimoptions(@fmincon);
options = optimoptions(options, 'Display', 'iter');

[kappa,fval,exitflag,output] = fmincon(fun,kappa0,A,b,Aeq,beq,lb,ub,nonlcon,options)

warning on