function [gamma,W] = softContactModel2D(q, qd, qdd, angleDeg, mdisc, bsurface,Mb,Mk)
%% Position States Vector: q = [theta;z_disc] x numsamples
%% Parameter Vector: p = [Ipulley;kpulley;bpulley;lambda(i)]; %lambda(i) for each sample 

%rows are position states, columns are samples:
[dim,numSamples] = size(q);

% Error checking: check if position state dimension is proper
assert(dim==2,'q needs to be 2 dimensional!')
assert(size(q,1)==size(qd,1),'q and qd do not have same row dimension!')
assert(size(q,2)==size(qd,2),'q and qd do not contain same number of samples!')
assert(size(qd,1)==size(qdd,1),'qd and qdd do not have same row dimension!')
assert(size(qd,2)==size(qdd,2),'qd and qdd do not contain same number of samples!')

%gravity
g = 9.81;
%angle of inclined plane
beta = angleDeg * pi/180; % angle of the sliding platform for the disk

%load urdf model
r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

theta = q(1,:)';
thetad = qd(1,:)';
thetadd = qdd(1,:)';

%z = q(2,:)';
zd = qd(2,:)';
zdd = qdd(2,:)';

J = zeros(numSamples,2);
qForConstraint = [q(1,:);...
                  zeros(1,numSamples);...
                  q(2,:)];

for i = 1:numSamples
    [~, dlength] = r.position_constraints{1}.eval(qForConstraint(:,i));
    J(i,:) = dlength([1,3]);
end

zeroVec = zeros(numSamples,1);
oneVec = ones(numSamples,1);
polyfun = @(base,exponent) base.^exponent;
%calculate the polynomial basis
Phib = bsxfun(polyfun,theta,(0:Mb));
Phik = bsxfun(polyfun,theta,(0:Mk));

W1 = [thetadd, thetad .* Phib,         theta.* Phik,          diag(-J(:,1))]; 
W2 = [zeroVec, zeros(numSamples,Mb+1),   zeros(numSamples,Mk+1),  diag(-J(:,2))];
W = [ W1;...
      W2];
       

gamma1 = zeroVec;
gamma2 = - bsurface* zd - mdisc * zdd - mdisc * g * sin(beta)  ;
gamma = [gamma1;
         gamma2];
  

%W = diag([100*ones(4,1);ones(numSamples,1)]);

% Solve Least Squares for parameters:
%p = inv(Wmat'* Wmat) * Wmat' * Gamma;

%Transpose
%p = p';
end