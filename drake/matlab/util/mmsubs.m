function q = mmsubs(p,x,v)
% Implements multiple column msubs.
%
% INPUTS:
%   p  -  m-by-n msspoly 
%   x  -  k-by-1 free msspoly 
%   v  -  k-by-1 real double 
%
% OUTPUT:
%   q  -  m-by-n msspoly
%
q = p;
for i=1:size(p,2)
  q(:,i) = msubs(p(:,i),x,v);
end
end