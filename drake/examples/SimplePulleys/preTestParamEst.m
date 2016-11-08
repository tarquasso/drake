% author Robert katzschmann 9/01/2016
tmp = addpathTemporary(fullfile(pwd,'..'));

Ts = 0.1;
obj =  PlanarRigidBodyManipulator('tensionWParams.urdf');
options.model = 'dynamic';
options.method = 'nonlinprog';
options.print_result = 'noprint';

%% Initialize

checkDependency('spotless');

nq = obj.getNumPositions;
nu = obj.getNumInputs;

p_orig = double(obj.getParams());
np = length(p_orig);

[pmin,pmax] = obj.getParamLimits();
assert(all(pmin>=0));  % otherwise I'll have to subtract it out from the coordinates
%  not hard, just not implmented yet. %% TODO: Implement allowing negative parameter
%  values

% Used often
isDynamic = strcmp(options.model,'dynamic');
isEnergetic = strcmp(options.model,'energetic');

%check if position contraints exist and then count those
if(~isempty(obj.position_constraints))
  isPositionConstrained = true;
  nc = numel(obj.position_constraints);
else
  isPositionConstrained = false;
  nc = 0;
end


%%   Step 1: Extract data
if (false)
  % populate A and b matrices from iddata
  % todo: make the processing of q,qd,qdd more robust
  qMeasured = nq - nc; %measured constraints are state constraints minus position constraints
  Ts = get(data,'Ts');
  t_data = get(data,'SamplingInstants')';
  x_data = get(data,'OutputData')';
  q_data = x_data(1:qMeasured,:);
  qd_data = x_data(qMeasured+(1:qMeasured),:);
  u_data = get(data,'InputData')';
  dt_data = diff(t_data);
  if isDynamic
    qdd_data = x_data(2*qMeasured+(1:qMeasured),:);
  end
  s_data = sin(q_data);
  c_data = cos(q_data);
  xobs = [q_data;qd_data];
end

%%   Step 2: Formulate error models
% Initialize Variables
q=msspoly('q',nq);
s=msspoly('s',nq);
c=msspoly('c',nq);
qt=TrigPoly(q,s,c);
qd=msspoly('qd',nq);
qdd=msspoly('qdd',nq);
u=msspoly('u',nu);
t=msspoly('t',1);

% Set up known parameters
p=obj.getParamFrame.getPoly;
pobj = setParams(obj,p); %Using model that was parsed in, overwriting numeric parameters with msspoly

% using parameterized model pobj to calculate manipulator equations that
% contain the parameters
[H,C,B] = manipulatorDynamics(pobj,qt,qd);


if isDynamic || isEnergetic
  if isDynamic
    % Formulate equation error from equations of motion
    if(nu > 0)
      err = H*qdd + C - B*u;
    else
      err = H*qdd + C;
    end
    
  elseif isEnergetic %TODO: make formulation independent of Acrobot
    % If need memory efficiency, can cut down on # of msspoly
    % this is just a bit more readable
    % Initialize msspoly variables for expressing dE = E(t_2)-E(t_1)
    dt=msspoly('dt',1);
    
    q1=msspoly('qo',nq);
    s1=msspoly('so',nq);
    c1=msspoly('co',nq);
    qt1=TrigPoly(q1,s1,c1);
    qd1=msspoly('qdto',nq);
    
    q2=msspoly('qf',nq);
    s2=msspoly('sf',nq);
    c2=msspoly('cf',nq);
    qt2=TrigPoly(q2,s2,c2);
    qd2=msspoly('qdtf',nq);
    
    % Formulate energy equations
    [T1,U1] = energy(pobj,[qt1;qd1]); %not revised yet
    [T2,U2] = energy(pobj,[qt2;qd2]);
    
    % ACROBOT-SPECIFIC FORMULATION - TODO: MUST CHANGE
    % Need to formulate energy dissipation from AcrobotPlant class
    dE = (B*u-[p(1);p(2)].*qd1)'*qd1*dt; %todo: make generic
    err = (T1+U1)-(T2+U2)+dE;
    
  end
  
  %% Add Constraints if those exist
  if(isPositionConstrained)
    % add position constraints to err matrix
    for i = 1:nc
      [~,jacobian] = pobj.position_constraints{i}.eval(qt);
      %append with the jacobian
      err = [err,jacobian'];
    end
  end
  
  
  % Isolate parameters from error equations
  [lp,M,Mb,lin_params,beta] = identifiableParameters(getmsspoly(err),p); % posynomial lumped params
  % [lp, M, Mb] = linearParameters(getmsspoly(err),p); % monomial lumped params
  nlp = length(lp);
  lp_orig = double(subs(lp,p,p_orig));
  lumped_params = msspoly('lp',nlp);
  % now err=M*lp+Mb and lperr=M*lumped_params+Mb;
  
  if isDynamic
    ndata = length(t_data);
    variables = [q;s;c;qd;qdd;u];
    data_variables = [q_data;s_data;c_data;qd_data;qdd_data;u_data];
    M_data = reshape(msubs(M(:),variables,data_variables)',nq*ndata,nlp);
    Mb_data = reshape(msubs(Mb,variables,data_variables)',nq*ndata,1);
  elseif isEnergetic
    variables = [q1;s1;c1;qd1;q2;s2;c2;qd2;u;dt];
    data_variables = [q_data(:,1:end-1);s_data(:,1:end-1);c_data(:,1:end-1);qd_data(:,1:end-1);...
      q_data(:,2:end);s_data(:,2:end);c_data(:,2:end);qd_data(:,2:end);u_data(:,1:end-1);dt_data];
    M_data = msubs(M(:),variables,data_variables)';
    Mb_data = msubs(Mb,variables,data_variables)';
  end
elseif strcmp(options.model,'simerr')
  % Using MATLAB sym because need to represent rational functions
  % Perhaps a better way?
  [xdot,dxdot,ps,qs,qds,us,ts] = pobj.dynamicsSym(t,[qt;qd],u);
  f = [xdot;dxdot];
end