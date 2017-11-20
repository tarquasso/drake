%% General todos: rewrite with fsolve

clear all
close all

%global L alpha beta D Phi
L = 100; % [m] Beam Length
R = 0.35; % [m] Radius Beam
Dia = R * 2; % [m] Diameter of manipulator, assumed constant for now

A = pi * Dia^2 / 4;
I1 = A^2 / (pi *4); % [m^4]
I3 = 2 * I1;

alpha = 1.345; % [Nm^2] twisting stiffness
beta = 0.789; % [Nm^2]

E = alpha/I1;
G_paper = beta/I3;

rho_lin_paper = 1; %[kg/m]


Ealt = 3.3827e7; % 5e5 or 592949 Pa Dragon Skin 30 https://www.smooth-on.com/products/dragon-skin-30/

nu = 0.5;  % Poission ratio [-]
G = E / (2*(1+nu));  % Shear modulus. E = 2G(1+ν) 

%M_h = 1;
%T_h = 1;

alphaAlt =  E * I1
betaAlt = G * I3



D = 3; % [m] Slack
Phi = 27* 2 * pi; % [rad] Twist

%D = 3/27; % [m] Slack
%Phi = 3 * 2 * pi; % [rad] Twist


mhofth = @(th) (sqrt(4*th .* (1 - (D^2*pi^2*th)/(L^2*4))));
thofmh = @(mh) ((mh./(2*cos(1/4*(Phi-(2*pi*mh)/(beta/alpha))))).^2);


%%
thmax = 450;
mhmax = 21.2;
varth = linspace(0,thmax,5000)';
varmh = linspace(0,mhmax,5000)';

% plot roots
figure(1)
plot(mhofth(varth),varth)
hold on
plot(varmh,thofmh(varmh))
xlabel('m_h')
ylabel('t_h')
axis([0 mhmax 0 thmax])

%plot(15.6834, 73.4830,'*')
%plot(15.4037,  380.0260,'*')

% find solutions in a one dimensional way

g = @(mh) (mh - mhofth(thofmh(mh)));

%% find more solutions
it = 3;
intval = zeros(it,2);
mh0 = zeros(it,1);
th0 = zeros(it,1);

intval(1,:) = [4.046, 4.105];
intval(2,:) = [8.287, 8.291];
intval(3,:) = [9.7, 9.99];
intval(4,:) = [15.40, 15.41];
intval(5,:) = [15.68, 15.69];
intval(6,:) = [20.24, 20.38];

for i =1:it
mh0(i) = fzero(g,intval(i,:));
th0(i) = thofmh(mh0(i));
figure(1)
plot(mh0(i), th0(i),'*')
hold on
figure(2) 
plotCurve(mh0(i),th0(i),L)
hold on
end
figure(2)
legend('1','2','3','4','5','6')


%% CONTINUE WORKING ON THE INTERVALS TO FIND SOLUTIONS!
%intervals = [0, 0.5,0.8, 1.158, 1.17, 1.187, 1.76, 2.137];
%interval = [18.99 19.39];
%mh0 = fzero(g,intervals(1,4))
%th0 = thofmh(mh0)



   
%% 

%DoverL = D/L;

m_h = @(M_h) M_h * L   / (2 * pi   * alpha);
t_h = @(T_h) T_h * L^2 / (4 * pi^2 * alpha);

DoverLFcn = @(M_h,T_h) (sqrt(4 / (pi^2 *t_h(T_h)) * (1 - m_h(M_h)^2 / (4 *t_h(T_h)))));
PhiFcn = @(M_h,T_h) (2* pi * m_h(M_h) / (beta/alpha) + 4* acos(m_h(M_h)/(2*sqrt(t_h(T_h)))));


F = @(x) ([D/L-DoverLFcn(x(1),x(2)), Phi - PhiFcn(x(1),x(2))]);
guess = [1.2,2];


%sols = fsolve(@dlandphi,guess);
sols = fsolve(F,guess);

% results are:
M_h = sols(1);%1.3017510114772594996354481051657;
T_h = sols(2);%2.0178798356198609814539591089399;
%m_h(M_h) %=15.4037
%t_h(T_h) %=380.0260

%% solve non-dimensional

DoverLFcn2 = @(m_h,t_h) (sqrt(4 / (pi^2 *t_h) * (1 - m_h^2 / (4 *t_h))));
PhiFcn2 = @(m_h,t_h) (2* pi * m_h / (beta/alpha) + 4* acos(m_h/(2*sqrt(t_h))));


F2 = @(x) ([D/L-DoverLFcn2(x(1),x(2)), Phi - PhiFcn2(x(1),x(2))]);
guess2 = [15,380];
guess2 = [5.28,6.969]; % ->   15.6834 - 0.0000i  73.4830 + 0.0000i

guess2 = [0.8,0.3]; % ->   15.40 + 0.0000i   380.03 + 0.0000i
guess2 = [20.78, 177.7]; %->   15.4037  380.0260

guess2 = [21.2,214];
guess2 = [15.4037,  380.0260];
guess2 = [26,76.92]; %->[15.6834, 73.4830]

guess2 =  [15.6834, 73.4830];

 guess2 =  [20,137];
 guess2 =  [19.15,128.5];
 
options = optimoptions('fsolve','Display','iter');
sols2 = fsolve(F2,guess2,options)


% results are:
m_h = sols2(1);%1.3017510114772594996354481051657;
t_h = sols2(2);%2.0178798356198609814539591089399;


%%
  
dlandphi(sols)

DoverLCalc = eval(subs(sqrt(4 / (pi^2 *t_h) * (1 - m_h^2 / (4 *t_h)))))
PhiCalc = eval(subs(2* pi * m_h / (beta/alpha) + 4* acos(m_h/(2*sqrt(t_h)))))

%%

function plotCurve(m_h,t_h,L)
N = 1000;
sh = linspace(-0.5,0.5,N)'; % sh = s/L - 0.5

comp1 = eval(subs(L * (1/(2*pi*t_h) * sqrt(4 * t_h - m_h^2) * sech(pi * sh * sqrt(4 * t_h - m_h^2)))));

req_i =  eval(subs(comp1 .* sin(m_h*pi*sh)));

req_j = eval(subs(- comp1 .* cos(m_h*pi*sh)));

req_k = eval(subs(L * (sh - 1/(2*pi*t_h) * sqrt(4 * t_h - m_h^2) * tanh(pi * sh * sqrt(4 * t_h - m_h^2)))));

plot3(req_i,req_j,req_k)
xlabel('i')
ylabel('j')
zlabel('k')
end
