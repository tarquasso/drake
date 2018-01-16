function F = dlandphi( x )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global L alpha beta D Phi

M_h = x(1);
T_h = x(2);

m_h = M_h * L   / (2 * pi   * alpha);
t_h = T_h * L^2 / (4 * pi^2 * alpha);

F(1) = D/L - sqrt(4 / (pi^2 *t_h) * (1 - m_h^2 / (4 *t_h)));
F(2) = Phi - (2* pi * m_h / (beta/alpha) + 4* acos(m_h/(2*sqrt(t_h))));

end

