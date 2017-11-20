%% General todos: rewrite with fsolve

clear all
close all

DoverL = 0.03;
Phi = 5 * 2 * pi; % [rad] Twist
nu = 0.5;
betaoveralpha = 1/(1+nu);

mhofth = @(th) (sqrt(4*th .* (1 - (DoverL^2*pi^2*th)/4)));
thofmh = @(mh) ((mh./(2*cos(1/4*(Phi-(2*pi*mh)/(betaoveralpha))))).^2);


%%
thmax = 450;
mhmax = 21.2;
varth = linspace(0,thmax,50000)';
varmh = linspace(0,mhmax,20000)';

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
it = 100;
intval = zeros(it,2);
mh0 = zeros(it,1);
th0 = zeros(it,1);

%intval(1,:) = [0, 1];
ic = 1;
% %intval(ic,:) = [0.66, 0.67]; ic=ic+1;
% intval(ic,:) = [0.67, 0.68]; ic=ic+1;
% 
% %intval(ic,:) = [1.9,2.0]; ic=ic+1;
% intval(ic,:) = [2.0,2.1]; ic=ic+1;
% 
% %intval(ic,:) = [3.28,3.334]; ic=ic+1;
% intval(ic,:) = [3.334,3.385]; ic=ic+1;

%lower branch
%intval(ic,:) = [4.5,4.67]; ic=ic+1;
intval(ic,:) = [4.67,4.9]; ic=ic+1;

% upper branch
%intval(ic,:) = [5.279,5.28]; ic=ic+1; %th going up
intval(ic,:) = [5.3875,5.388]; ic=ic+1; %th going down

%lower branch
%intval(ic,:) = [5.9,5.99]; ic=ic+1;
intval(ic,:) = [5.99,6.09]; ic=ic+1;

% upper branch
%intval(ic,:) = [6.599,6.6]; ic=ic+1; %th going up
intval(ic,:) = [6.735,6.736]; ic=ic+1; %th going down

legtext = cell(it,1);
sh = cell(it,1);
req = cell(it,1);

chosenIndices = 1:1:ic-1;

for i =chosenIndices
mh0(i) = fzero(g,intval(i,:));
th0(i) = thofmh(mh0(i));
figure(1)
plot(mh0(i), th0(i),'*')
hold on
figure(2) 
[sh{i},req{i}] = plotBucklingCurve(mh0(i),th0(i));
hold on
legtext{i} = strcat('sol', num2str(i));
end
figure(2)
legend(legtext{chosenIndices})


%% CONTINUE WORKING ON THE INTERVALS TO FIND SOLUTIONS!
%intervals = [0, 0.5,0.8, 1.158, 1.17, 1.187, 1.76, 2.137];
%interval = [18.99 19.39];
%mh0 = fzero(g,intervals(1,4))
%th0 = thofmh(mh0)



   
%% 

%DoverL = D/L;

% m_h = @(M_h) M_h * L   / (2 * pi   * alpha);
% t_h = @(T_h) T_h * L^2 / (4 * pi^2 * alpha);
% 
% DoverLFcn = @(M_h,T_h) (sqrt(4 / (pi^2 *t_h(T_h)) * (1 - m_h(M_h)^2 / (4 *t_h(T_h)))));
% PhiFcn = @(M_h,T_h) (2* pi * m_h(M_h) / (beta/alpha) + 4* acos(m_h(M_h)/(2*sqrt(t_h(T_h)))));
% 
% 
% F = @(x) ([D/L-DoverLFcn(x(1),x(2)), Phi - PhiFcn(x(1),x(2))]);
% guess = [1.2,2];
% 
% 
% %sols = fsolve(@dlandphi,guess);
% sols = fsolve(F,guess);
% 
% % results are:
% M_h = sols(1);%1.3017510114772594996354481051657;
% T_h = sols(2);%2.0178798356198609814539591089399;
% %m_h(M_h) %=15.4037
% %t_h(T_h) %=380.0260
% 
% %% solve non-dimensional
% 
% DoverLFcn2 = @(m_h,t_h) (sqrt(4 / (pi^2 *t_h) * (1 - m_h^2 / (4 *t_h))));
% PhiFcn2 = @(m_h,t_h) (2* pi * m_h / (beta/alpha) + 4* acos(m_h/(2*sqrt(t_h))));
% 
% 
% F2 = @(x) ([D/L-DoverLFcn2(x(1),x(2)), Phi - PhiFcn2(x(1),x(2))]);
% guess2 = [15,380];
% guess2 = [5.28,6.969]; % ->   15.6834 - 0.0000i  73.4830 + 0.0000i
% 
% guess2 = [0.8,0.3]; % ->   15.40 + 0.0000i   380.03 + 0.0000i
% guess2 = [20.78, 177.7]; %->   15.4037  380.0260
% 
% guess2 = [21.2,214];
% guess2 = [15.4037,  380.0260];
% guess2 = [26,76.92]; %->[15.6834, 73.4830]
% 
% guess2 =  [15.6834, 73.4830];
% 
%  guess2 =  [20,137];
%  guess2 =  [19.15,128.5];
%  
% options = optimoptions('fsolve','Display','iter');
% sols2 = fsolve(F2,guess2,options)
% 
% 
% % results are:
% m_h = sols2(1);%1.3017510114772594996354481051657;
% t_h = sols2(2);%2.0178798356198609814539591089399;
% 
% 
% %%
%   
% dlandphi(sols)
% 
% DoverLCalc = eval(subs(sqrt(4 / (pi^2 *t_h) * (1 - m_h^2 / (4 *t_h)))))
% PhiCalc = eval(subs(2* pi * m_h / (beta/alpha) + 4* acos(m_h/(2*sqrt(t_h)))))
% 
% %%


