function [s,beta,z,x] = calculateEndMoment(length,moment)
global B My
close all
My = moment; %N

L = length; % [m] Beam Length
R = 0.015;
D = R * 2; % [m] Diameter of manipulator, assumed constant for now
E = 5.0e5; % 592949 Pa Dragon Skin 30 https://www.smooth-on.com/products/dragon-skin-30/
I = pi * D^4/64; % [m^4] area moment of inertia circle along radial axis
B = E*I;
meshsize = 1000;
nDiscretization = 10000;

% Calculation of the initial guess : 
% β(0) = 0
% and
% dβ/ds(L) = My/B 
y_initial_guess = [0 My/B];

solinit = bvpinit(linspace(0,L,meshsize),y_initial_guess);

sol = bvp4c(@twoode,@twobc,solinit);

s = linspace(0,L,nDiscretization)';

y = deval(sol,s);

%Transform the resulting frame
%s = sInv';
beta = y(1,:)';
dbetads = y(2,:)';

figure
plot(s,dbetads,'.')
xlabel('s (free end at s=L)[m]')
ylabel('d\beta/ds [rad/m]')
title(['End Moment with My = ',num2str(My),'N'])
axis ([0 L -inf inf])

figure
plot(s,beta,'.')
xlabel('s (free end at s=L)[m]')
ylabel('\beta [rad]')
title(['End Moment with My = ',num2str(My),'N'])
axis ([0 L -inf inf])

% integrate z-axis
z = cumtrapz(s,cos(abs(beta)));

figure
plot(z,beta,'.')
xlabel('z [m]')
ylabel('\beta [rad]')
title(['End Moment with My = ',num2str(My),'N'])
axis ([0 L -inf inf])

% integrate x-axis
x = cumtrapz(s,sin(abs(beta)));

figure
%plot x over z
plot(z,x,'.')

xlabel('z [m]')
ylabel('Deflection x [m]')
axis square %([0 inf -inf inf])
axis equal
title(['End Moment with My = ',num2str(My),'N'])
savefig('data/end_load_analytical.fig')

end
%--------------------------------------
function dydx = twoode(x,y)
% y(1) is y and y(2) is y'
% B d²β/ds² =0
% d²β/ds² = 0
% y'' = 0
dydx = [ y(2);... 
        0 ];
end
%--------------------------------------
function res = twobc(ya,yb)
global B My
% dβ/ds(0) = 0
% β(L) = 0
res = [ ya(1) ; 
        yb(2) - My/B];
end
