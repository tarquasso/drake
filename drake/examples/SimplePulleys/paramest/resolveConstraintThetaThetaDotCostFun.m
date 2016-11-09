function [g,dg] = resolveConstraintThetaThetaDotCostFun(q, qd)

r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');

[cable_length,J,ddlength] = r.position_constraints{1}.eval(q);

lengthdot = J * qd;

f = [cable_length - r.position_constraints{1}.lb;...
     lengthdot];

g = dot(f,f);
   
df = [ J(1),                      0    ; ...
       ddlength(1:3)*qd,          J(1)];

dg = 2*f'*df;     

end