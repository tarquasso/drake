function ps = msspoly2sym(xm,xs,pm)

% SPOT to symbolic matlab converter
% flips an msspoly polynomial, pm, which depends on variables xm, and
% outputs the sym polynomial, ps, in terms of the syms xs.

typecheck(xm,'msspoly');
typecheck(xs,'sym');
typecheck(pm,'msspoly');

[x,p,M] = decomp(pm);

N = size(xs,2);
n = size(p,2);

if n == 0 || ~any(any(p))
    ps = repmat(double(pm),1,N);
    return;
end

% decomp places the variables in a different order.
% we must sort xs to match this order.
[~,xn]  = isfree(x);  % extract ids of x.
[~,xmn] = isfree(xm); % extract ids of xm.
perm = mss_match(xmn,xn); % compute permutation.
xs = xs(perm);

% M = vpa(M,3);   warning('dramatically reducing precision here as a test of numerics');

ps=reshape(M*prod(repmat(xs',size(p,1),1).^p,2),size(pm));



