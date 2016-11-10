function p = ordinaryLeastSquares(q, qd, qdd, angleDeg, mdisc)
%% Position States Vector: q = [theta;z_disc] x numsamples
%% Parameter Vector: p = [Ipulley;kpulley;bpulley;bsurface;lambda]; 

r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

% columns are the states
% rows are samples
[dim,numSamples] = size(q);
if(dim~=2)
  error('q needs to be 2 dimensional!')
end

theta = q(1,:)';
thetad = qd(1,:)';
thetadd = qdd(1,:)';

z = q(2,:)';
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

Ymat1 = [thetadd, thetad,  theta,   zeroVec, diag(-J(:,1))]; 
Ymat2 = [zeroVec, zeroVec, zeroVec, zd,      diag(-J(:,2))];

Ymat = [ Ymat1;...
         Ymat2];
       
g = 9.81;
beta = angleDeg * pi/180; % angle of the sliding platform for the disk

Gamma1 = zeroVec;
Gamma2 = - mdisc * ( zdd + g * sin(beta) ) ;
Gamma = [Gamma1;
         Gamma2];

%W = diag([100*ones(4,1);ones(numSamples,1)]);

p = inv(Ymat'* Ymat) * Ymat' * Gamma;

p = p';
end