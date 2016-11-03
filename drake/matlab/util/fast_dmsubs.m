function [q,fn]=fast_dmsubs(p,x,v)
% function q=dmsubs(p,x,v)
%
%
% Data matrix substitution.
%
%
% INPUTS:
%   p  -  m-by-1 msspoly 
%   x  -  k-by-1 free msspoly 
%   v  -  k-by-n real double 
%
% OUTPUT:
%   q  -  m-by-n double
%
% DESCRIPTION: q(:,i) is the result of substituting v(:,i) for x in p
% p must be an msspoly in x alone.

if nargin < 3, error('Three arguments required.'); end
if ~isa(p,'msspoly'), error('First argument must be an msspoly.'); end

m = size(p,1);
if size(p,2) ~= 1, error('First argument must be m-by-1.'); end

[f,xn] = isfree(x);
if ~f, error('Second argument must be free msspoly.'); end
k = size(x,1);
if size(x,2) ~= 1, error('Second argument must by k-by-1'); end

if size(v,1) ~= k, 
    error(['Second/Third arguments must have the same number of rows']);
end

if ~isa(v,'double'), error('Third argument must be a double.'); end
    

% xsym = sym('x', size(x),'real');
% psym = msspoly2sym(x,xsym,p);
% fn = matlabFunction(psym,'Vars',xsym);
% arg = num2cell(v);
% 
% q = zeros(length(p), size(v,2));
% for i = 1:size(v,2)
%   q(:,i) = fn(arg{:,i});
% end 

[pows,coeffs] = decomp_ordered(p,x);

N = size(v,2);
n = size(pows,2);

if n == 0 || ~any(any(pows))
    q = repmat(double(p),1,N);
    return;
end
q = zeros(length(p), size(v,2));
for i = 1:size(v,2)
  q(:,i) = coeffs*prod(repmat(v(:,i)',size(coeffs,2),1).^pows,2);
end 
end
