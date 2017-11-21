function tang = calcTangent(req,s)
%CALCTANGENT calculate the tangent of the motion
%   Detailed explanation goes here
ds = mean(diff(s));
tang = zeros(size(req));
for i = 1:3
tang(:,i) = gradient(req(:,i),ds);
end
% Alternative:
%tang = diff(req)/ds(2:end);
end

