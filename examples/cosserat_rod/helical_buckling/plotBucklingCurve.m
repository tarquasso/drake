function [sh,r] = plotBucklingCurve(m_h,t_h,shmax)
N = 5000;
shmax = shmax;
shmin = -shmax;
sh = linspace(shmin,shmax,N)'; % sh = s/L - 0.5
L = 1; % length is normalized to 1

comp1 = eval(subs(L * (1/(2*pi*t_h) * sqrt(4 * t_h - m_h^2) * sech(pi * sh * sqrt(4 * t_h - m_h^2)))));

req_i =  eval(subs(comp1 .* sin(m_h*pi*sh)));

req_j = eval(subs(- comp1 .* cos(m_h*pi*sh)));

req_k = eval(subs(L * (sh - 1/(2*pi*t_h) * sqrt(4 * t_h - m_h^2) * tanh(pi * sh * sqrt(4 * t_h - m_h^2)))));

r = [req_i,req_j,req_k];

plot3(req_k,-req_j,req_i)
grid on
xlabel('k')
ylabel('j')
zlabel('i')

%plot3(req_i,req_j,req_k)
%xlabel('i')
%ylabel('j')
%zlabel('k')
end