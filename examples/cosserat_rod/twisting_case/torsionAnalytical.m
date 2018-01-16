function [x,theta_last] = torsionAnalytical(N,Tmax)
%% Constants
beam_length = 0.2; % [m] Beam Length
R = 0.015;
D = R * 2; % [m] Diameter of manipulator, assumed constant for now
J = pi * D^4/32; % [m^4] Torsional Constant of a Circle
E = 5.0e5; % 592949 Pa Dragon Skin 30 https://www.smooth-on.com/products/dragon-skin-30/
nu = 0.5; % for Rubber, see https://en.wikipedia.org/wiki/Poisson%27s_ratio
G = E/((1+nu)*2); % isotropic Pascal Shear Modulus Rubber according to http://www.engineeringtoolbox.com/modulus-rigidity-d_946.html

%% Simulation Inputs
steps = 40; % steps to plot
T = linspace(0,Tmax,steps); % Applied Torque

%% Calculation
% For a beam of uniform cross-section along its length:
segment_length_half = (beam_length / N)/2; 
beam_length_center_last_segment = beam_length - segment_length_half;
theta = (T * beam_length_center_last_segment) / (J * G); % angle of twist

%last time step
x = [0,linspace(segment_length_half,beam_length_center_last_segment,N)]'; % positions along arm
theta_last = (Tmax * x) / (J * G); % angle of twist along arm for last time step

if 0
%% Plotting
figure

plot(T,theta*180/pi,'ro')
xlabel('Torque [Nm]');
ylabel('Tip Angle \theta [deg]');

figure
circleLine3(0,0,0,D/2,theta(1))  

%Connecting line of angles
xline = [D/2 * cos(theta(1)); D/2 * cos(theta(end))];
yline = [D/2 * sin(theta(1)); D/2 * sin(theta(end))];
zline = [0;L];
hold on
h = plot3(xline,yline,zline,'r');
hold off

%Connecting line of angles
xline = [0;0];
yline = [0;0];
zline = [0;L];
hold on
h = plot3(xline,yline,zline,'m--');
hold off

% for i=1:length(theta)
    circleLine3(0,0,L,D/2,theta(end))  
% end
az = 100;
el = 45;
view(az, el);

xlabel('x')
ylabel('y')
zlabel('z')
axis equal
end
end
