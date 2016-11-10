function f = paramEstCostFun2D(p, q, qd, qdd, angleDeg, mdisc)
%% Position States Vector: q = [theta;z_disc] x numsamples
%% Parameter Vector: p = [Ipulley;kpulley;bpulley;bsurface]; 

r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

% columns are the states
% rows are samples

theta = q(1,:);
thetad = qd(1,:);
zd = qd(2,:);

g = 9.81;
beta = angleDeg * pi/180; % angle of the sliding platform for the disk

Ipulley = p(1);
kpulley = p(2);
bpulley = p(3);
bsurface = p(4);

[dim,numSamples] = size(q);

qForConst = [q(1,:);zeros(1,numSamples);q(2,:)];

psi = NaN(dim*numSamples,1);
for i = 1:numSamples

    H = diag([Ipulley, mdisc]);
    C = [ bpulley*thetad(i) + kpulley*theta(i); ...
          mdisc*g*sin(beta) + bsurface*zd(i)];
    
    
    [~, J, d2phidq2] = r.position_constraints{1}.eval(qForConst(:,i));
    J = [J(1),J(3)];
    d2phidq2 = [d2phidq2(1), d2phidq2(7); ...
                d2phidq2(3), d2phidq2(9)];
    Jdot = (d2phidq2*qd(:,i))';
    
    Hinv = inv(H);
    Hinvtilde = J*Hinv*J';
    Htilde = inv(Hinvtilde);
    
    temp = H*qdd(:,i) + (eye(dim) - J'*Htilde*J*Hinv)*C + J'*Htilde*Jdot*qd(:,i);
    
    psi((dim*(i-1)+1):dim*i) = temp;

end

f = dot(psi, psi);

end