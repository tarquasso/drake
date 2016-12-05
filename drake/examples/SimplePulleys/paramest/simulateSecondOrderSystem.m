function sol = simulateSecondOrderSystem(g,b,k,m,tf,z0,zd0)
syms z(t)
bm = b/m; 
km = k/m;
[V] = odeToVectorField(diff(z, 2) == - g - bm * diff(z) - km*z);
M = matlabFunction(V,'vars', {'t','Y'});
timeRange = [0 tf];
initialConditions = [z0 zd0];
sol = ode45(M,timeRange,initialConditions);

end