function f = paramEstCostFun2D(p, q, qd, qdd, angleDeg, mdisc, bsurface)
%% Position States Vector: q = [theta;z_disc] x numsamples
%% Parameter Vector: p = [Ipulley;kpulley;bpulley;bsurface]; 

%extract dimension of q
[dim,numSamples] = size(q);

%check if dimesnion is only 2, this code does not support the 3d position
%state.
assert(dim == 2)

r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

% columns are the states
% rows are samples

%Extract states in readable form
theta = q(1,:);
thetad = qd(1,:);
zd = qd(2,:);

%gravity
g = 9.81;
% sliding angle for inclined table
beta = angleDeg * pi/180; % angle of the sliding platform for the disk

%parameters to be estimated
Ipulley = p(1);
kpulley = p(2);
bpulley = p(3);

%generate constraint with higher dimension (3 instead of 2)
qUsedForConstraint = [q(1,:);zeros(1,numSamples);q(2,:)];

% 
psi = NaN(dim*numSamples,1);
for i = 1:numSamples
    
    %mass matrix:
    H = diag([Ipulley, mdisc]);
    %dissipative, spring, gravity forces:
    C = [ bpulley*thetad(i) + kpulley*theta(i); ...
          mdisc*g*sin(beta) + bsurface*zd(i)];
    
    %TODO: Add a means of adjusting the cablelength as a parameter... 
    %calculate the constraint using plant r and the plant definition of r
    [~, J, d2phidq2] = r.position_constraints{1}.eval(qUsedForConstraint(:,i));
    % extract jacobian just for two states:
    J = [J(1),J(3)];
    % Caluclate Hessian Matrix for the dims = 2 case
    d2phidq2 = [d2phidq2(1), d2phidq2(7); ...
                d2phidq2(3), d2phidq2(9)];
    %Calculate Jdot using hessian matrix and velocity:
    Jdot = (d2phidq2*qd(:,i))';
    
    %mass matrix inverse
    Hinv = inv(H);
    % Argument of Moore-Penrose Pseudo-Inverse
    Hinvtilde = J*Hinv*J';
    % Moore-Penrose Pseudo-Inverse
    Htilde = inv(Hinvtilde);
    
    %solution for lambda combined with dynamics equation: 
    dynamicsErr = H*qdd(:,i) + (eye(dim) - J'*Htilde*J*Hinv)*C + J'*Htilde*Jdot*qd(:,i);
    
    % build it up for all samples, two rows at a time
    psi((dim*(i-1)+1):dim*i) = dynamicsErr;

end

%calculate cost
f = dot(psi, psi);

%no derivative df provided...
end