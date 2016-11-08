%% Parse in Optitrack Capture Data

% So far only working for Data Set 5
filename = '~/soft_modeling_repo/dev/tracking/data/set5/16-May-2015 20_46_41.mat';

% Set 5 Touching point: 55.8mm - manually measured from the Optitrack
tp = 0.0558;
%Static spring stretching point: 52.8 mm
expStartTime = 5; %seconds
expEndTime = 8; %seconds

[xuls,tuls] = parse_tension_experiment_data(filename,tp,expStartTime,expEndTime);

%%

% adjust in height to the tensionWParamsExp.urdf coordinates

% fit it to a polynomial to extract q, qd, qdd

% 
% q = rand(10,3);
% qd = 10*rand(10,3);
% qdd = 100*rand(100,3);


% p0 = rand(4,1); % simulation hand tuning

%p0 = 



% fun = @(p) paramEstCostFun(p, q, qd, qdd);
% options = optimoptions(@fmincon);
% options = optimoptions(options, 'Display', 'iter');
% 
% [x,fval] = fmincon(fun, p0, [],[], [], [], 0.01*ones(4,1), 1000*ones(4,1),[], options);