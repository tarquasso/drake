function f = paramEstCostFun(p, q, qd, qdd)

r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

theta = q(:,1);
thetad = qd(:,1);
thetadd = qdd(:,1);
xdd = qdd(:,2);
zdd = qdd(:,3);

g = 9.81;
beta = 30*pi/180;

Ipulley = p(1);
mball = p(2);
k = p(3);
b = p(4);

psi = NaN(length(theta),1);
for i = 1:length(theta)
    H = diag([Ipulley, mball, mball]);
    C = [b*thetad(i) + k*theta(i); 0; mball*g*cos(beta)];
    
    % [H,C,~] = manipulatorDynamics(r, q, qd);
    
    [~, J, d2phidq2] = r.position_constraints{1}.eval(q(i,:)');
    Jdot = (reshape(d2phidq2,length(q(i,:)'),[])*qd(i,:)')';
    
    Hinv = inv(H);
    Hinvtilde = J*Hinv*J';
    Htilde = inv(Hinvtilde);
    
    psi(3*i-2:3*i) = H*qdd(i,:)' + (eye(3) - J'*Htilde*J*Hinv)*C + J'*Htilde*Jdot*qd(i,:)';
end

f = dot(psi, psi);

end