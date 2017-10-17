%% General todos: rewrite with fsolve

clear all
close all

L = 100; % [m] Beam Length
R = 0.015; % [m] Radius Beam
Dia = R * 2; % [m] Diameter of manipulator, assumed constant for now

E = 5.0e5; % 592949 Pa Dragon Skin 30 https://www.smooth-on.com/products/dragon-skin-30/

nu = 0.5;  % Poission ratio [-]
G = E / (2*(1+nu));  % Shear modulus. E = 2G(1+ν) 
A = pi * Dia^2 / 4;
I1 = A^2 / (pi *4);
I3 = 2 * I1;

M_h = 1;
T_h = 1;

alphaAlt =  E * I1;
betaAlt = G * I3;

alpha = 1.345; % [Nm^2] twisting stiffness
beta = 0.789; % [Nm^2]
D = 3; % [m] Slack
Phi = 27* 2 * pi; % [rad] Twist

DoverL = D/L;

syms M_h T_h
m_h = M_h * L   / (2 * pi   * alpha);
t_h = T_h * L^2 / (4 * pi^2 * alpha);

eqn1 = DoverL == sqrt(4 / (pi^2 *t_h) * (1 - m_h^2 / (4 *t_h)));
eqn2 = Phi == 2* pi * m_h / (beta/alpha) + 4* acos(m_h/(2*sqrt(t_h)));

eqns = [eqn1, eqn2];

sols = solve(eqns,[M_h,T_h]);

% results are:
M_h = 1.3017510114772594996354481051657;
T_h = 2.0178798356198609814539591089399;

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

