function [phat,simulation_error,estimated_delay,exitflag] = parameterEstimation(obj,data,varargin)
%
% Parameter estimation algorithm for manipulators
%
% For the 'dynamic' mode attempts to minimize objective
%  \[ \min_p  | H(q,p)qddot - C(q,qd,p) - B(q,qd,p)*u |_2^2 \]
% by extracting an affine representation using lumped-parameters and then
% running nonlinear least-squares.
%
% For the 'energetic' mode attempts to minimize objective
%  \[ \min_p  | Edot*dt + E(t1) - E(t2) |_2^2 \]
% by extracting an affine representation using lumped-parameters and then
% running nonlinear least-squares.
%
% For the 'simerr' mode attempts to minimize objective
% \[ \min_p \sum((qsim(i)-qobs(i))'C(qsim(i)-qobs(i))) \]
%
% Restrictions:
%   H,C,and B must be trig/poly in q,qd and polynomial in p.
%   All params p must be lower bounded (general constraints are not
%   implemented yet, but would be easy if they are affine)
%   so far, I require full-state feedback
%
% Algorithm:
%   Step 1: Extract data
%      Parse data object and save state as variables q,qd,qdd (if needed)
%   Step 2: Extract lumped-parameters
%      Use TrigPoly + spotless to parse H,C,B and extract unique monomial
%      coefficients in p.
%   Step 3: Nonlinear Least-squares
%      Create a nonlinear least squares program to estimate the parameters
%      from either the lumped-parameter formulation (in the case of
%      'dynamic' and 'energetic') or the trajectory optimization ('simerr')
%
%
% @param data an instance of iddata (from the system identification
% toolbox; see 'help iddata' for more info) containing the data to
% be used for estimation.
%
% @option print_result determines if the function will print the results
% 'noprint'     = no print,
% 'printEst'    = only print estimated
% 'printAll'	= print estimated and original
%
% @option model determines which model to use for estimation
% 'dynamic'     = Uses the manipulator equations to create an affine
%                 representation of the dynamics using lumped-parameters
%                 uses the second derivative of position - requires qdd
% 'energetic'   = Uses energy equation to estimate the evolution of kinetic
%                 and potential energies based on input torques and damping
%                 losses
% 'simerr'      = minimize the sum of differences between simulated state
%                 and observed state over all timesteps:
%                 \[ \min_p \sum((qsim(i)-qobs(i))'C(qsim(i)-qobs(i))) \]
%                 given a cost matrix C
%
% @option method determines which method to use for estimation
% 'nonlinprog'  = nonlinear least squares (LS) to solve problem
% 'lpprog'      = linear LS on lumped params then nonlinear LS to recover
%                 original parameters (only for 'dynamic' and 'energetic')

%% handle options
if (nargin>2 && isstruct(varargin{1})) options=varargin{1};
else options=struct(); end

% If parameters missing, use default values
if (~isfield(options,'print_result'))
  options.print_result='noprint';
end
if (~isfield(options,'model'))
  options.model='dynamic';
end
if (~isfield(options,'method'))
  options.method='nonlinprog';
end
if (~isfield(options,'C'))
  options.C=eye(2*obj.num_positions);
end
if (~isfield(options,'symbolicClass'))
  options.symbolicClass='msspoly'; %other option is 'sym'
end
if ~(strcmp(options.model,'dynamic') ||...
    strcmp(options.model,'energetic') ||...
    strcmp(options.model,'simerr'))
  error('Model not recognized')
end
if ~(strcmp(options.method,'nonlinprog') ||...
    strcmp(options.method,'linprog'))
  error('Method not recognized')
end


matlabSymsFlag = true;

fprintf('Initializing ...\n');
%% Initialize
if (getOutputFrame(obj)~=getStateFrame(obj))
  error('Only full-state feedback is implemented so far');
end
checkDependency('spotless');

nq = obj.getNumPositions;
nu = obj.getNumInputs;

p_orig = double(obj.getParams());
np = length(p_orig);

[pmin,pmax] = getParamLimits(obj);
assert(all(pmin>=0));  % otherwise I'll have to subtract it out from the coordinates
%  not hard, just not implmented yet. %% TODO: Implement allowing negative parameter
%  values

%check if position contraints exist and then count those
if(~isempty(obj.position_constraints))
  isPositionConstrainted = true;
  nc = numel(obj.position_constraints);
else
  isPositionConstrainted = false;
  nc = 0;
end

% Used often
isDynamic = strcmp(options.model,'dynamic');
isEnergetic = strcmp(options.model,'energetic');
useMatlabSym = strcmp(options.symbolicClass,'sym');

fprintf('Extracting data ...\n');
%%   Step 1a: Extract data
if (nargin>1)
  % populate A and b matrices from iddata
  % todo: make the processing of q,qd,qdd more robust
  qMeasured = nq - nc; %measured constraints are state constraints minus position constraints
  Ts = get(data,'Ts');
  t_data = get(data,'SamplingInstants')';
  x_data = get(data,'OutputData')';
  q_data = x_data(1:qMeasured,:);
  n = size(q_data,2);
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

%%   Step 1b: Reconstruct missing states q(1) and qd(1)
%If there is a constraint, reconstruct missing q(1) qd(1)
if(isPositionConstrainted)
  %TODO: z0 by using resolve constraint
  q_known = q_data; %nMeasuredx1xn
  qd_known = qd_data; %nMeasuredx1xn
  
  for i=1:size(q_data,2)
    %use resolveconstraint instead
    nlprog = NonlinearProgram(2);
    constraint = FunctionHandleConstraint(zeros(nc*2,1),zeros(nc*2,1),...
      @(q_uk,qd_uk) cableLengthConstraint(obj,q_known(:,i),qd_known(:,i),q_uk,qd_uk));
    %nlprog.x_name
    nlprog = nlprog.addConstraint(constraint,{1,2});
    z = nlprog.solve(z0); %some more return arguments like the status
    z0 = z; %reassign current as the previous state for the next program run
    % TODO: write the reconstructed data to the full dataset;
  end
  % ToDo: use polynomial fitting to get from (q(1), qd(1)) to qdd(1)
  polynomialOrder = 5;
  [polynomialFactors,S,mu] = polyfit(timeSteps,q1Data,polynomialOrder);
  %calculate residual between q data and qd data
  
  %%   Step 1c) Calculate Constraint J = dphi/dq = dlength
  Jdata = zeros(nq,size(qData,2));
  for i = 1:size(qData,2)
    [~,Jdata(:,i)] = obj.position_constraints{i}.eval(qDataAll); %%TODO: fgix this by running iteration
  end
end

fprintf('Formulating error models ...\n');
%%   Step 2: Formulate error models
% Initialize Variables
q=msspoly('q',nq);
s=msspoly('s',nq);
c=msspoly('c',nq);
if(useMatlabSym)
  %q = sym('q%d',[nq,1],'real');
  qtm=TrigPoly(q,s,c);
  qt = getsym(qtm); %is there a trigpoly function for syms?
  qd = sym('qd',[nq,1],'real');
  %qd = msspoly2sym(qd,qds,qd);
  qdd = sym('qdd',[nq,1],'real');
  %qdd = msspoly2sym(qdd,qdds,qdd);
  u = sym('u',[nu,1],'real');
  %u = msspoly2sym(u,us,u);
  t = sym('t',[1,1],'real');
else % using msspoly
  qt=TrigPoly(q,s,c);
  qd=msspoly('qd',nq);
  qdd=msspoly('qdd',nq);
  u=msspoly('u',nu);
  t=msspoly('t',1);
end

% Set up known parameters
% TODO: Fix this so it also works for Matlab Symbolic
%if(useMatlabSym)
%  paramsSym = sym(obj.getParamFrame().getCoordinateNames(), 'real');
%else
paramsSym = obj.getParamFrame.getPoly; %remove all the unnecessary parameters
%end
if ~isempty(paramsSym)
  pobj = obj.setParams(paramsSym); %Using model that was parsed in, overwriting numeric parameters with msspoly
end

if(isPositionConstrainted)
  % msspoly describing the unknown constraint force
  lambda = msspoly('lamb',1); %constraint force
  % add the constraint force as a parameter to the paramsSym object
  paramsSym = [paramsSym; lambda];
  % treat J as an msspoly that gets replaced later
  J = msspoly('j',nq);
end

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
    if(isPositionConstrainted)
      err = err - J*lambda;
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
    %TODO this needs implementation within rigidbodymanipulator
    [T1,U1] = energy(pobj,[qt1;qd1]);
    [T2,U2] = energy(pobj,[qt2;qd2]);
    
    % ACROBOT-SPECIFIC FORMULATION - TODO: MUST CHANGE
    % Need to formulate energy dissipation from AcrobotPlant class
    % TODO: make generic or fit for tension example
    dE = ([paramsSym(1);paramsSym(2)].*qd1)'*qd1*dt;
    err = (T1+U1)-(T2+U2)+dE;
    
  end
  
  % Add Constraints if those exist - this method does not work because of cableLength calculations
  %if(isPositionConstrainted)
  % add position constraints to err matrix
  %for i = 1:nc
  %[parametricPositionConstraint,parametricVelocityConstraint] = ...
  % pobj.position_constraints{i}.eval(qt);
  %err = [err;parametricPositionConstraint;parametricVelocityConstraint];
  %q1 and q1d into lp?
  %end
  %end
  
  % Isolate parameters from error equations
  fprintf('Running ID Params ...\n');
%   profile on
  [lp,M,Mb,lin_params,beta] = identifiableParameters(getmsspoly(err),paramsSym); % polynomial lumped params
  % [lp, M, Mb] = linearParameters(getmsspoly(err),p); % monomial lumped params
  
  nlp = length(lp);
  lp_orig = double(subs(lp,paramsSym,p_orig));
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
  fprintf('creating simerror functions ...\n');
  if matlabSymsFlag
    % Use built-in Matlab symbolic
    [xdot,dxdot,ps,qs,qds,us,ts] = dynamicsSym(pobj,t,[qt;qd],u);
    f = [xdot;dxdot];
    dfdx = matlabFunction(jacobian(f*ts,[qs;qds]),'Vars',[ps;qs;qds;us;ts]);
    dfdp = matlabFunction(jacobian(f*ts,ps),'Vars',[ps;qs;qds;us;ts]);
%     dfdx = @(pval,qval,qdval,uval,tval) dfdxMat(subsref(num2cell([pval;qval;qdval;uval;tval]'),struct('type','{}','subs',{{':'}})));
%     dfdp = @(pval,qval,qdval,uval,tval) dfdpMat(subsref(num2cell([pval;qval;qdval;uval;tval]'),struct('type','{}','subs',{{':'}})));
    nonlinfun = @(px) simerr(obj,xobs,u_data,px,t_data,options.C,dfdx,dfdp,'sym');
  else
    %TODO: This needs debugging, since it runs much slower than matlab syms
    % MSSPOLY is a faster substitution method adopted from Michael Posa
    % This implementation, which is slow, uses the quotient/chain rule to 
    % calculate the derivative of Hinv*A w.r.t x
    % d(inv(H)*A)/dx = -inv(H)*dH/dx*inv(H)+inv(H)*dA/dx
    %
    vars = [paramsSym;q;s;c;qd;u;t];
    xvar = [q;qd];
    A = B*u-C;
    [Apows,Acoeffs] = decomp_ordered(getmsspoly(A),vars);
    Adecomp = struct('msspoly',getmsspoly(A),'pows',Apows,'coeffs',Acoeffs,'vars',vars);
    [Hpows,Hcoeffs] = decomp_ordered(getmsspoly(H),vars);
    Hdecomp = struct('msspoly',getmsspoly(H),'pows',Hpows,'coeffs',Hcoeffs,'vars',vars);
    
    [dHxdecomp,dAxdecomp] = dHinvA(H,A,xvar,vars);
    dfdx = @(pval,qval,qdval,uval,tval) dHinvAdxFun(pval,qval,qdval,uval,tval,Hdecomp,Adecomp,dHxdecomp,dAxdecomp,qd,xvar);
    [dHpdecomp,dApdecomp] = dHinvA(H,A,paramsSym,vars);
    dfdp = @(pval,qval,qdval,uval,tval) dHinvAdxFun(pval,qval,qdval,uval,tval,Hdecomp,Adecomp,dHpdecomp,dApdecomp,qd,paramsSym);
    nonlinfun = @(px) simerr(obj,xobs,u_data,px,t_data,options.C,dfdx,dfdp);
  end
end

fprintf('Running Nonlinear least squares ...\n');
%%   Step 3: Nonlinear least-squares estimation
% Nonlinear least-squares solver
prog = NonlinearProgram(np);
prog=prog.addConstraint(BoundingBoxConstraint(pmin,pmax),1:np);

%% Add geometric constraint
% xCurrentResolved = resolveConstraints(obj,xCurrent);
% theta = xCurrentResolved(1);
% thetadot = xCurrentResolved(4);
% prog=prog.addStateConstraint(BoundingBoxConstraint(theta,theta),1);

nx = length(xobs(:,1));

if strcmp(options.model,'simerr')
  prog=prog.addCost(FunctionHandleObjective(np,nonlinfun),1:np);
  prog=prog.setSolverOptions('snopt','IterationsLimit',50000);
  prog=prog.setSolverOptions('snopt','MajorIterationsLimit',50000);
  prog=prog.setSolverOptions('snopt','MajorOptimalityTolerance',1.0e-5);
elseif strcmp(options.method,'nonlinprog')
  % Only nonlinear least squares
  nonlinfun = @(x) nonlinerr(x,lp,paramsSym,M_data,Mb_data);
  prog=prog.addCost(FunctionHandleObjective(np,nonlinfun),1:np);
elseif strcmp(options.method,'linprog')
  % Least squares -> Nonlinear least squares on lumped parameter solution
  lp_est = -pinv(full(M_data))*Mb_data;
  prog=prog.addQuadraticCost(eye(np),p_orig,1:np);
  lpconstraint_handle = @(x) lpconstraint_fun(x,lp,paramsSym);
  lpconstraint = FunctionHandleConstraint(lp_est,lp_est,nlp,lpconstraint_handle);
  prog=prog.addConstraint(lpconstraint,1:np);
end
[x,simulation_error,exitflag] = prog.solve(p_orig);

phat = Point(getParamFrame(obj),x);

%% Computing the simulation error
% err_orig = M_data*lp_orig + Mb_data;
% sqerr_orig = sum(err_orig'*err_orig);
if strcmp(options.method,'nonlinprog') || strcmp(options.method,'linprog')
  simulation_error = simerr(obj,xobs,u_data,x,t_data,options.C);
end

%%   Step 4: Print Results.
if strcmp(options.print_result,'printest')
  coords = getCoordinateNames(getParamFrame(obj));
  fprintf('\nParameter estimation results:\n\n');
  fprintf('  Param  \tEstimated\n');
  fprintf('  -----  \t---------\n');
  for i=1:length(coords)
    fprintf('%7s  \t%8.2f\n',coords{i},phat(i));
  end
elseif strcmp(options.print_result,'printall')
  coords = getCoordinateNames(getParamFrame(obj));
  fprintf('\nParameter estimation results:\n\n');
  fprintf('  Param  \tOriginal\tEstimated\n');
  fprintf('  -----  \t--------\t---------\n');
  for i=1:length(coords)
    fprintf('%7s  \t%8.2f\t%8.2f\n',coords{i},p_orig(i),phat(i));
  end
end
%TODO: calculate estimated_delay
estimated_delay = 0;

end

function [f,df] = lpconstraint_fun(x,lp,p)
f = msubs(lp,p,x);
dlpdp = diff(lp,p);
% There must be a better way to msubs a matrix with spotless
df = zeros(size(dlpdp,2));
for i=1:size(dlpdp,2)
  df(:,i) = msubs(dlpdp(:,i),p,x);
end
end

function [f,df] = nonlinerr(x,lp,p,M_data,Mb_data)
sqrterr = M_data*(msubs(lp,p,x))+Mb_data;
f = sqrterr'*sqrterr;
dlpdp = diff(lp,p);
% There must be a better way to msubs a matrix with spotless
dlpdp_val = zeros(size(dlpdp,2));
for i=1:size(dlpdp,2)
  dlpdp_val(:,i) = msubs(dlpdp(:,i),p,x);
end
df = 2*(M_data*(msubs(lp,p,x))+Mb_data)'*M_data*dlpdp_val;
end

function dfdx = dHinvAdxFun(p_data,q_data,qd_data,u_data,t_data,H,A,dHdx,dAdx,qd,x)
%
% Uses the quotient/chain rule to calculate the derivative of Hinv*A w.r.t x
% d(inv(H)*A)/dx = -inv(H)*dH/dx*inv(H)+inv(H)*dA/dx
%
  vars = A.vars;
  in = [p_data;q_data;sin(q_data);cos(q_data);qd_data;u_data;t_data];
  
  Hval = double(mmsubs(H.msspoly,vars,in));
  Hvalinv = pinv(Hval);
  Aval = double(mmsubs(A.msspoly,vars,in));

  dHinvAdx = zeros(length(q_data),length(x));
  for i=1:length(x)
    dAval = double(mmsubs(dAdx{i}.msspoly,vars,in));
    dHval = double(mmsubs(dHdx{i}.msspoly,vars,in));
    dHinvAdx(:,i) = -Hvalinv*(dHval*Hvalinv*Aval+dAval);
  end
  dqdx = diff(qd,x);
  dqdxval = double(mmsubs(dqdx,vars,in));
  dfdx = t_data*[dqdxval;dHinvAdx];
end

function [F,dF] = simerr(obj,xobs,utraj,p,t,C,varargin)
newobj = obj.setParams(p);
x0 = xobs(:,1);
nq = length(x0)/2;
error = false;
try
  xtraj = computeTraj(newobj,x0,utraj,t);
  xdiff = xtraj - xobs;
  F = sum(sum(xdiff.*(C*xdiff),1),2);
catch err
  % The system parameters are unstable, output infinite error
  if strcmp(err.identifier,'MATLAB:svd:matrixWithNaNInf')
    error = true;
    F = Inf;
  else rethrow(err);end
end
if (nargout>1)
  if ~error
    if ~(nargin>7); error('Requires input dfdx and dfdp'); end
    dfdx = varargin{1};
    dfdp = varargin{2};
    dt = diff(t);
    N = length(dt);
    I = eye(length(x0));
    dx = cell(1,N);
    if length(varargin)>2 && strcmp(varargin{3},'sym')
      args = num2cell([p;x0;utraj(:,1);dt(1)]'); dx{1} = dfdp(args{:});
    else
      dx{1} = dfdp(p,x0(1:nq),x0(nq+1:end),utraj(:,1),dt(1));
    end
    dF = xdiff(:,2)'*C*dx{1};
    for i=2:(N)
      if length(varargin)>2 && strcmp(varargin{3},'sym')
        args = num2cell([p;xtraj(:,i);utraj(:,i);dt(i)]');dfdxi = dfdx(args{:});dfdpi = dfdp(args{:});
      else
        dfdxi = dfdx(p,xtraj(1:nq,i),xtraj(nq+1:end,i),utraj(:,i),dt(i)); dfdpi = dfdp(p,xtraj(1:nq,i),xtraj(nq+1:end,i),utraj(:,i),dt(i));
      end
      dx{i} = (I+dfdxi)*dx{i-1}+dfdpi;
      dF = dF + xdiff(:,i+1)'*C*dx{i};
    end
    dF = 2*dF;
  else
    dF = -Inf(1,length(p));
  end
end
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

function [f,df] = cableLengthConstraint(obj, q_known,qd_known,q_uk,qd_uk)
%call eval within cableLength.m
[cable_length,dlength,ddlength] = obj.positionConstraints([q_uk;q_known]);
lengthdot = dlength*[qd_uk;qd_known];
dlengthdot = [reshape(ddlength,[],numel([q_uk;q_known]))*[qd_uk;qd_known] dlength];
f = [cable_length - obj.pulley_length; lengthdot]; %find obj.pulley_length
df = [dlength zeros(size(dlength)); dlengthdot];
end