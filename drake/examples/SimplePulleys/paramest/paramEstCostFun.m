function f = paramEstCostFun(p, q, qd, qdd, angleDeg, mdisc)
%% Position States Vector: q = [theta;x_disc;z_disc] x numsamples
%% Parameter Vector: p = [Ipulley;kpulley;bpulley]; 

r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

% columns are the states
% rows are samples

theta = q(1,:);
thetad = qd(1,:);
xd = qd(2,:);
zd = qd(3,:);

g = 9.81;
beta = angleDeg * pi/180; % angle of the sliding platform for the disk

Ipulley = p(1);
kpulley = p(2);
bpulley = p(3);
bsurface = p(4);

[dim,numSamples] = size(q);
dimUsed = 2;
psi = NaN(dimUsed*numSamples,1);
for i = 1:numSamples

    H = diag([Ipulley, mdisc, mdisc]);
    C = [ bpulley*thetad(i) + kpulley*theta(i); ...
          bsurface*xd(i);...
          mdisc*g*cos(beta) + bsurface*zd(i)];
    
    % q=msspoly('q',nq);
    % s=msspoly('s',nq);
    % c=msspoly('c',nq);
    % qt=TrigPoly(q,s,c);
    % qd=msspoly('qd',nq);
    % qdd=msspoly('qdd',nq);
    % u=msspoly('u',nu);
    % p=r1.getParamFrame.getPoly;
    % pr1 = setParams(r1,p);
    % [H,C,B] = manipulatorDynamics(r,qt,qd);
    % % we do not need B here
    
    [~, J, d2phidq2] = r.position_constraints{1}.eval(q(:,i));
    Jdot = (reshape(d2phidq2,length(q(:,i)),[])*qd(:,i))';
    
    Hinv = inv(H);
    Hinvtilde = J*Hinv*J';
    Htilde = inv(Hinvtilde);
    
    temp = H*qdd(:,i) + (eye(3) - J'*Htilde*J*Hinv)*C + J'*Htilde*Jdot*qd(:,i);
    
    if (dimUsed == 2)
      temp = temp([1,3]);
    elseif (dimUsed == 3)
      temp = temp;
    elseif (dimUsed == 1)
       temp = temp(3);
    end
    
    psi((dimUsed*(i-1)+1):dimUsed*i) = temp;

end

f = dot(psi, psi);

end