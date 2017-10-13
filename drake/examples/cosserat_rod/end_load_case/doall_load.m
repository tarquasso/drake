close all
clear all

% works with: commit: ?
% tag is cosserat_end_load

%exp_time = 0.12;
Fmax = 2.0;  % [N] maximum force
L = 0.2; % [m] length of beam
%calculate numerical solution using bvp4c:
[s,beta,z,x] = calculateEndLoadUpdated(L,Fmax);

% define number of link elements
linkElements = [5,10,20,40,80,160];
numTests = length(linkElements);
filenames = {'data/poses_Fmax2_0_N05.dat',...
    'data/poses_Fmax2_0_N10.dat',...
    'data/poses_Fmax2_0_N20.dat',...
    'data/poses_Fmax2_0_N40.dat',...
    'data/poses_Fmax2_0_N80.dat',...
    'data/poses_Fmax2_0_N160.dat'};
assert(numTests == length(filenames))

% initialize values
% x_final = cell(numTests,1);
% z_final = cell(numTests,1);
% beta_final = cell(numTests,1);
% beta_final_analytical = cell(numTests,1);
% x_final_analytical = cell(numTests,1);
beta_l2norm = zeros(numTests,1);
x_l2norm = zeros(numTests,1);
delta_s = zeros(numTests,1);
steadyStateErrorBeta = zeros(numTests,1);
steadyStateErrorX = zeros(numTests,1);

for i = 1:numTests
    NBodies = linkElements(i); %just number of bodies
    N = NBodies + 1; % Number of bodies (including the world)
    
    drakedata = importdata(filenames{i});
    lengthOfData = length(drakedata);
    
    is=lengthOfData-N+2;
    ie=lengthOfData;
    
    t_steady_num = drakedata(is:ie,1); %time
    z_steady_num = drakedata(is:ie,4); %coordinate vertically down
    beta_steady_num = drakedata(is:ie,5); %rotation
    x_steady_num = drakedata(is:ie,2); %deflection
    
    %     z_final{i} = z_steady_num;
    %     x_final{i} = x_steady_num;
    %     beta_final{i} = beta_steady_num;
    
    %interpolation
    delta_s(i) = L/NBodies;
    delta_s_alt = drakedata(3,4)-drakedata(2,4);
    tol = eps(0.5);
    assert(abs(delta_s(i) - delta_s_alt) < tol)
    %s_num_alt = drakedata(2:N,4);
    % or:
    s_num = ((delta_s(i)/2):delta_s(i):(L-delta_s(i)/2))';
    
    % x_0(s_i) - analytical solution at s_i
    z0 = interp1(s,z,s_num);
    x0 = interp1(s,x,s_num);
    z_eps_sq = (z_steady_num-z0).^2;
    
    beta0 = interp1(s,beta,s_num);
    beta_eps_sq = (beta_steady_num-beta0).^2;
    
    beta_epssq = beta_eps_sq;
    beta_onesegment = beta_epssq*delta_s(i);
    beta_l2norm(i) = sqrt(1/L*sum(beta_onesegment));
    
    x_eps_sq = (x_steady_num-x0).^2;
    zx_epssq = z_eps_sq+x_eps_sq;
    zx_onesegment = zx_epssq*delta_s(i);
    x_l2norm(i) = sqrt(1/L*sum(zx_onesegment));
    
    t_num = drakedata(N:N:end,1);
    dt_num = diff(t_num);
    
    tFinal = t_num(end);
    
    T = 0.4719-0.1608;
    relevantIndices = find(t_num>(tFinal-T));
    
    betas_num = drakedata(N:N:end,5);
    betas_last_oscillations = betas_num(relevantIndices(1):end);
    d_betas_last_osc = diff(betas_last_oscillations);
    [~,dbetas_finalpartialIdx] = max(abs(d_betas_last_osc));
    beta_finalIdx = relevantIndices(1)+dbetas_finalpartialIdx;
    steadyStateErrorBeta(i) = (max(betas_last_oscillations)-min(betas_last_oscillations))/2;
    
    x_num = drakedata(N:N:end,2);
    x_last_oscillations = betas_num(relevantIndices);
    d_x_last_osc = diff(x_last_oscillations);
    [~,dx_finalpartialIdx] = max(abs(d_x_last_osc));
    x_finalIdx = relevantIndices(1)+dx_finalpartialIdx;
    steadyStateErrorX(i) = (max(x_last_oscillations)-min(x_last_oscillations))/2;
       
%     figure
%     plot(t_num,betas_num);
%     xlabel('time [sec]')
%     ylabel('\beta [rad]')
%     title('\beta over time')
    
    % figure
    % %subplot(221)
    % h1=plot(z_steady_num,beta_steady_num,'g*',z,beta,'r-');
    % %axis([0 0.8 -0.7 0.1]); axis square
    % %set(h1, 'MarkerSize', 18, 'Color', 'k', 'LineWidth', 1);
    % %set(gca, 'FontName', 'Times', 'FontSize', 16)
    % xlabel('z [m]')
    % ylabel('beta [radians]')
    % legend('numerical','analytical','Location','NorthWest')
    % title('Bending Angle for each segment at maximum force level')
    %
    % figure
    % h1=plot(z_steady_num,x_steady_num,'g*',z,x,'r-');
    % %axis([0 0.8 -0.7 0.1]); axis square
    % %set(h1, 'MarkerSize', 18, 'Color', 'k', 'LineWidth', 1);
    % xlabel('z [m]')
    % ylabel('x [m]')
    % legend('numerical','analytical','Location','NorthWest')
    % title('Bending Displacement at maximum force level')
    
    
    
end

% SteadyStateError Beta
figure
%subplot(211)
h1 = loglog(delta_s,steadyStateErrorBeta,'.-');
set(h1, 'MarkerSize', 16, 'Color', 'k', 'LineWidth', 1);
hold on
% loglog(delta_s,delta_s.^0.5*1e-2,'-s')
%loglog(delta_s,delta_s.^(1.0)*10^(-6.5),'k--')
%loglog(delta_s,delta_s.^(1.05)*10^(-0.14),'k--')
grid
set(gca, 'FontName', 'Times', 'FontSize', 16)
xlabel('$\Delta s$ [m]','FontName', 'Times', 'FontSize', 16,'Interpreter','latex')
ylabel('Steady State Error on $\beta$ [rad]', 'FontSize', 16,'Interpreter','latex')
%legend('l2norm','O(N^1)','O(N^{1.5})')
%title('L2 Norm for \beta')
print('plots/end_load_beta_steady_state_error','-depsc')

% SteadyStateError X
figure
%subplot(211)
h1 = loglog(delta_s,steadyStateErrorX,'.-');
set(h1, 'MarkerSize', 16, 'Color', 'k', 'LineWidth', 1);
hold on
% loglog(delta_s,delta_s.^0.5*1e-2,'-s')
%loglog(delta_s,delta_s.^(1.0)*10^(-6.5),'k--')
%loglog(delta_s,delta_s.^(1.05)*10^(-0.14),'k--')
grid
set(gca, 'FontName', 'Times', 'FontSize', 16)
xlabel('$\Delta s$ [m]','FontName', 'Times', 'FontSize', 16,'Interpreter','latex')
ylabel('Steady State Error on $x$ [m]', 'FontSize', 16,'Interpreter','latex')
%legend('l2norm','O(N^1)','O(N^{1.5})')
%title('L2 Norm for \beta')
print('plots/end_load_x_steady_state_error','-depsc')

% L2 norm
figure
%subplot(211)
h1 = loglog(delta_s,beta_l2norm,'.-');
set(h1, 'MarkerSize', 16, 'Color', 'k', 'LineWidth', 1);
hold on
% loglog(delta_s,delta_s.^0.5*1e-2,'-s')
loglog(delta_s,delta_s.^(1.0)*10^(-0.00),'k--')
loglog(delta_s,delta_s.^(1.05)*10^(0.21),'k--')
grid
set(gca, 'FontName', 'Times', 'FontSize', 16)
xlabel('$\Delta s$ [m]','FontName', 'Times', 'FontSize', 16,'Interpreter','latex')
ylabel('$\mathnormal{l}_2$ for $\beta$ [rad]', 'FontSize', 16,'Interpreter','latex')
%legend('l2norm','O(N^1)','O(N^{1.5})')
%title('L2 Norm for \beta')
print('plots/end_load_beta_l2_norm','-depsc')

figure
%subplot(212)
h2 = loglog(delta_s,x_l2norm,'.-');
set(h2, 'MarkerSize', 16, 'Color', 'k', 'LineWidth', 1);
hold on
loglog(delta_s,delta_s.^(0.95)*10^(-1.32),'k--')
loglog(delta_s,delta_s.^(1.0)*10^(-1.12),'k--')
grid on
set(gca, 'FontName', 'Times', 'FontSize', 16)
xlabel('$\Delta s$ [m]','FontName', 'Times', 'FontSize', 16,'Interpreter','latex')
ylabel('$\mathnormal{l}_2$ for $x$ [m]', 'FontSize', 16,'Interpreter','latex')
%legend('l_2','O(N^1)','O(N^{1.5})')
%title('L2 Norm for x')
print('plots/end_load_x_l2_norm','-depsc')
