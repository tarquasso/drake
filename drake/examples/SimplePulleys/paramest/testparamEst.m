discRadius = 0.044230; %4.423cm of disc radius
spreadPulley = 0.29;

thetamin = -pi/2;
thetamax = pi/2;

xmin = -spreadPulley/4;
xmax= +spreadPulley/4;

zmin = 0; %pushing down by one radius length
zmax= discRadius; %at contact

numSamples= 10;
q = [rand(1,numSamples) * (thetamax-thetamin) + thetamin; ...
  rand(1,numSamples) * (xmax-xmin) + xmin;...
  rand(1,numSamples) * (zmax-zmin) + zmin];

qd = 10*rand(3,numSamples);
qdd = 100*rand(3,numSamples);

dimParams = 3;
min = 0.01;
max = 1000;
p0 = rand(dimParams,1) * (max-min)+min; % simulation hand tuning needed for this parameter
angleDeg = 36;
mdisc = 0.131;

fun = @(p) paramEstCostFun(p, q, qd, qdd, angleDeg, mdisc);
options = optimoptions(@fmincon);
options = optimoptions(options, 'Display', 'iter');

[pEstimated,fval] = fmincon(fun, p0, [],[], [], [], ...
           min*ones(dimParams,1), max*ones(dimParams,1),[], options);
         
IpulleyEst = pEstimated(1);
kpulleyEst = pEstimated(2);
bpulleyEst = pEstimated(3);

IpulleyEst,kpulleyEst,bpulleyEst