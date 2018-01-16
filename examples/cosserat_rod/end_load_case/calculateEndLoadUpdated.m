function [s,beta,z,x] = calculateEndLoadUpdated(length,force)
global B Nx
close all
Nx = force; %N

L = length; % [m] Beam Length
R = 0.015;
D = R * 2; % [m] Diameter of manipulator, assumed constant for now
E = 5.0e5; % 592949 Pa Dragon Skin 30 https://www.smooth-on.com/products/dragon-skin-30/
I = pi * D^4/64; % [m^4] area moment of inertia circle along radial axis
B = E*I;
meshsize = 10000;
nDiscretization = 100000;

% Calculation of the initial guess : 
% Nₓ cos(β) = 0 => β = pi/2
% and
% d²β/ds² = 0 => dβ/ds = 0 - actually anything could be picked for this...
y_initial_guess = [pi/2 0];

solinit = bvpinit(linspace(0,L,meshsize),y_initial_guess);

sol = bvp4c(@twoode,@twobc,solinit);

sInv = linspace(0,L,nDiscretization);

y = deval(sol,sInv);

%Transform the resulting frame
s = sInv';
beta = fliplr(y(1,:))';

% figure
% plot(s,beta,'.')
% xlabel('s (free end at s=L)[m]')
% ylabel('\beta [rad]')
% title(['End Load with Nx = ',num2str(Nx),'N'])
% axis ([0 L 0 inf])

% integrate z-axis
z = cumtrapz(s,cos(beta));

% figure
% plot(z,beta,'.')
% xlabel('z [m]')
% ylabel('\beta [rad]')
% title(['End Load with Nx = ',num2str(Nx),'N'])
% axis ([0 L 0 inf])

% integrate x-axis
x = cumtrapz(s,sin(beta));

% figure
% %plot x over z
% plot(z,x,'.')
% xlabel('z [m]')
% ylabel('Deflection x [m]')
% title(['End Load with Nx = ',num2str(Nx),'N'])
% savefig('data/end_load_analytical.fig')
% %axis ([0 inf -inf inf])

end
%--------------------------------------
function dydx = twoode(x,y)
global B Nx
% y(1) is y and y(2) is y'
% B d²β/ds² + Nₓ cos(β) =0
% d²β/ds² = - Nₓ/B cos(β)
% y'' = -Nₓ/B cos(y)
dydx = [ y(2);... 
        - Nx/B * cos(y(1)) ];
end
%--------------------------------------
function res = twobc(ya,yb)
% dβ/ds(0) = 0
% β(L) = 0
res = [ ya(2); 
        yb(1) ];
end
