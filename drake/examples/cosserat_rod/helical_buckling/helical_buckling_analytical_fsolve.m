%% General todos: rewrite with fsolve

clear all
close all

%global L alpha beta D Phi
L = 100; % [m] Beam Length
R = 0.015; % [m] Radius Beam
Dia = R * 2; % [m] Diameter of manipulator, assumed constant for now

E = 5.0e5; % 592949 Pa Dragon Skin 30 https://www.smooth-on.com/products/dragon-skin-30/

nu = 0.5;  % Poission ratio [-]
G = E / (2*(1+nu));  % Shear modulus. E = 2G(1+ν) 
A = pi * Dia^2 / 4;
I1 = A^2 / (pi *4);
I3 = 2 * I1;

%M_h = 1;
%T_h = 1;

alphaAlt =  E * I1;
betaAlt = G * I3;

alpha = 1.345; % [Nm^2] twisting stiffness
beta = 0.789; % [Nm^2]

D = 3; % [m] Slack
Phi = 27* 2 * pi; % [rad] Twist


mhofth = @(th) (sqrt(4*th .* (1 - (D^2*pi^2*th)/(L^2*4))));
thofmh = @(mh) ((mh./(2*cos(1/4*(Phi-(2*pi*mh)/(beta/alpha))))).^2);

thmax = 450;
mhmax = 21.2;
varth = linspace(0,thmax,1000)';
varmh = linspace(0,mhmax,1000)';


figure
plot(varth,mhofth(varth))
% xlabel('t_h')
% ylabel('m_h')
hold on


plot(thofmh(varmh), varmh)
xlabel('t_h')
ylabel('m_h')
axis([0 thmax 0 mhmax])

%DoverL = D/L;

m_h = @(M_h) M_h * L   / (2 * pi   * alpha);
t_h = @(T_h) T_h * L^2 / (4 * pi^2 * alpha);

DoverLFcn = @(M_h,T_h) (sqrt(4 / (pi^2 *t_h(T_h)) * (1 - m_h(M_h)^2 / (4 *t_h(T_h)))));
PhiFcn = @(M_h,T_h) (2* pi * m_h(M_h) / (beta/alpha) + 4* acos(m_h(M_h)/(2*sqrt(t_h(T_h)))));


F = @(x) ([D/L-DoverLFcn(x(1),x(2)), Phi - PhiFcn(x(1),x(2))]);
guess = [1.2,2];


%sols = fsolve(@dlandphi,guess);
sols = fsolve(F,guess);


%% 
% results are:
M_h = sols(1);%1.3017510114772594996354481051657;
T_h = sols(2);%2.0178798356198609814539591089399;
m_h(M_h)
t_h(T_h)

dlandphi(sols)

DoverLCalc = eval(subs(sqrt(4 / (pi^2 *t_h) * (1 - m_h^2 / (4 *t_h)))))
PhiCalc = eval(subs(2* pi * m_h / (beta/alpha) + 4* acos(m_h/(2*sqrt(t_h)))))

%%
N = 1000;
sh = linspace(-0.5,0.5,N)'; % sh = s/L - 0.5

comp1 = eval(subs(L * (1/(2*pi*t_h) * sqrt(4 * t_h - m_h^2) * sech(pi * sh * sqrt(4 * t_h - m_h^2)))));

req_i =  eval(subs(comp1 .* sin(m_h*pi*sh)));

req_j = eval(subs(- comp1 .* cos(m_h*pi*sh)));

req_k = eval(subs(L * (sh - 1/(2*pi*t_h) * sqrt(4 * t_h - m_h^2) * tanh(pi * sh * sqrt(4 * t_h - m_h^2)))));

figure
plot3(req_i,req_j,req_k)
xlabel('i')
ylabel('j')
zlabel('k')

