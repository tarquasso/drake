function f = paramEstCostFun(p, q, qd, qdd)

r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

% columns are the states
% rows are samples

theta = q(:,1);
thetad = qd(:,1);
thetadd = qdd(:,1);
xdd = qdd(:,2);
zdd = qdd(:,3);

g = 9.81;
beta = 30*pi/180; % angle of the sliding platform for the disk

Ipulley = p(1);
mball = p(2);
kpulley = p(3);
bpulley = p(4);
numSamples = length(theta);
dim = 2;

psi = NaN(dim*numSamples,1);
for i = 1:numSamples
    H = diag([Ipulley, mball, mball]);
    C = [ bpulley*thetad(i) + kpulley*theta(i); ...
          0;...
          mball*g*cos(beta)];
    
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
    
    [~, J, d2phidq2] = r.position_constraints{1}.eval(q(i,:)');
    Jdot = (reshape(d2phidq2,length(q(i,:)'),[])*qd(i,:)')';
    
    Hinv = inv(H);
    Hinvtilde = J*Hinv*J';
    Htilde = inv(Hinvtilde);
    
    temp = H*qdd(i,:)' + (eye(3) - J'*Htilde*J*Hinv)*C + J'*Htilde*Jdot*qd(i,:)';
    if (dim == 2)
       psi((dim*i-1):dim*i) = temp([1,3]);
    elseif (dim == 3)
      psi((dim*i-2):dim*i) = temp;
    end
end

f = dot(psi, psi);

end