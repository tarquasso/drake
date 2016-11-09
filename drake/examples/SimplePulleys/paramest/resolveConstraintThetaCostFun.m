% function [g,dg] = resolveConstraintThetaCostFun(q, qd)
function [g,dg] = resolveConstraintThetaCostFun(q)

r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

[cable_length,J,~] = r.position_constraints{1}.eval(q);

% lengthdot = J * qd;

% f = [cable_length - r.position_constraints{1}.lb;...
%      lengthdot];
% 
% g = dot(f,f);
%    
% df = [ J(1),                      0    ; ...
%        ddlength(1:3)*qd,          J(1)];
% 
% dg = 2*f'*df;     





f = cable_length - r.position_constraints{1}.lb;
df = J(1);

g = f^2;
dg = 2*f*df;

end