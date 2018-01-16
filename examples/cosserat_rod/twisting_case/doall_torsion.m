close all
clear all

% works with: commit: 2882c1ab1788ee63792ed3b58034c27dbf16d9e6  
% tag is cosserat_twist_case

exp_time = 0.12;
Tmax = 0.03;  % [Nm] maximum torque

n = 21;  % Number of bodies (including the world).
[x,theta_last] = torsionAnalytical(n-1,Tmax);


drakedata = importdata('data/poses_Tmax30mNm.dat');

nsteps = length(drakedata)/n;

% Plot final time profile.
figure
it=nsteps; 

is=(it-1)*n+1; 
ie=is+n-1;

x_num = drakedata(is:ie,4);
theta_last_num = drakedata(is:ie,5);
subplot(221)
h1=plot(x_num,theta_last_num,'g.',x,theta_last,'r*');
%axis([0 0.8 -0.7 0.1]); axis square
%set(h1, 'MarkerSize', 18, 'Color', 'k', 'LineWidth', 1);
set(gca, 'FontName', 'Times', 'FontSize', 16)
xlabel('x [m]', 'FontName', 'Times', 'FontSize', 16)
ylabel('theta [radians]', 'FontName', 'Times', 'FontSize', 16)
legend('numerical','analytical','Location','NorthWest')
title('Twist for each segment at maximum torque level')


delta_x = x-x_num;
delta_theta = theta_last-theta_last_num;

subplot(222)

h2=plot(x,delta_x,'b.');
%axis([0 0.8 -0.7 0.1]); axis square
%set(h2, 'MarkerSize', 18, 'Color', 'k', 'LineWidth', 1);
set(gca, 'FontName', 'Times', 'FontSize', 16)
xlabel('x [m]', 'FontName', 'Times', 'FontSize', 16)
ylabel('delta x [m]', 'FontName', 'Times', 'FontSize', 16)
title('Delta X (@Tmax)')

subplot(223)

h3=plot(x,delta_theta,'b.');
%axis([0 0.8 -0.7 0.1]); axis square
%set(h3, 'MarkerSize', 18, 'Color', 'k', 'LineWidth', 1);
set(gca, 'FontName', 'Times', 'FontSize', 16)
xlabel('x [m]', 'FontName', 'Times', 'FontSize', 16)
ylabel('delta theta [radians]', 'FontName', 'Times', 'FontSize', 16)
title('Delta \theta (@Tmax)')

% Plot final element y-position vs. time.
subplot(224)
ibody=n;
t = drakedata(ibody:n:end,1);
dt = diff(t);
indices = find (t<exp_time);


thetas = drakedata(ibody:n:end,5);
dthetas = diff(thetas);
v = dthetas./dt;
test = find (v<0.00001);
range = 1:test(1);
t = t(indices);
thetas = thetas(indices);
h4 = plot(t,thetas,'.');
%set(h4, 'Color', 'k', 'LineWidth', 2);
set(gca, 'FontName', 'Times', 'FontSize', 16)
xlabel('time [sec]', 'FontName', 'Times', 'FontSize', 16)
ylabel('theta [m]', 'FontName', 'Times', 'FontSize', 16)
title('\theta as T_max increases')

% figure
% plot(t(1:end-1),dt,'.')

% % Estimate error with analytical solutions.
% ell = L / (n-1);
% s=linspace(ell/2, L - ell/2, n-1);
% s = [0 s];
% theta = s/Rc;
% ys = -Rc + Rc * cos(theta);
% xs = Rc * sin(theta);
% 
% xnum = d(is:ie,4)';
% ynum = d(is:ie,3)';
% 
% L2norm = norm(xs-xnum) + norm(ys-ynum);
% 
% [e(1) h(1)] = load_and_compute_error('poses_5.dat', 5);
% [e(2) h(2)] = load_and_compute_error('poses_10.dat', 10);
% [e(3) h(3)] = load_and_compute_error('poses_20.dat', 20);
% [e(4) h(4)] = load_and_compute_error('poses_40.dat', 40);
% 
% figure(3)
% h3=loglog(h,e, 'o', h, (h.^2)/6,'k--');
% set(h3, 'MarkerSize', 6, 'Color', 'k', 'LineWidth', 1);
% set(gca, 'FontName', 'Times', 'FontSize', 16)
% xlabel('h [m]', 'FontName', 'Times', 'FontSize', 16)
% ylabel('L_2 [m^2]', 'FontName', 'Times', 'FontSize', 16)
