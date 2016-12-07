function sol = simulateSecondOrderSystem(g,b,k,Mb,Mk,m,tf,z0,zd0)
syms z(t)

if (Mb == 1)
[V] = odeToVectorField(diff(z, 2) == - g - (b(1)+b(2)*z)/m * diff(z) - (k(1)+k(2)*z)/m*z);
elseif (Mb >1 || Mk >1)
  error('not supported');
else
[V] = odeToVectorField(diff(z, 2) == - g - b/m * diff(z) - k/m*z);  
end
M = matlabFunction(V,'vars', {'t','Y'}); 
timeRange = [0 tf];
initialConditions = [z0 zd0];
sol = ode45(M,timeRange,initialConditions);
end