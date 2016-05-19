function [dHdecomp,dAdecomp] = dHinvA(H,A,dx,vars)
% 
% Outputs  decomposition of each required polynomial
if size(H,1) ~= size(A,1)
    error('Second argument must have the same height (first dimension).');
end

if size(A,2) ~= 1
    error('Second argument must by m-by-1 msspoly.');
end

if size(dx,2) ~= 1
    error('Third argument must by m-by-1 msspoly.');
end

%% Initialize variables
nx = length(dx);

% [Apows,Acoeffs] = decomp_ordered(A,vars);
% Adecomp = struct('pows',Apows,'coeffs',Acoeffs,'vars',vars);
% [Hpows,Hcoeffs] = decomp_ordered(H,vars);
% Hdecomp = struct('pows',Hpows,'coeffs',Hcoeffs,'vars',vars);

dAdecomp = cell(nx,1);
dHdecomp = cell(nx,1);
for i=1:nx
  xi = dx(i);
  Adxi = diff(A,xi);
  if isa(Adxi,'TrigPoly')
    Adxi = getmsspoly(Adxi);
  end
  Hdxi = H;
  for j=1:size(H,2)
    Hdxi(:,j) = diff(H(:,j),xi);
  end
  if isa(Hdxi,'TrigPoly')
    Hdxi = getmsspoly(Hdxi);
  end
  [Adxipows,Adxicoeffs] = decomp_ordered(Adxi,vars);
  dAdecomp{i} = struct('msspoly',Adxi,'pows',Adxipows,'coeffs',Adxicoeffs,'vars',vars);
  [Hdxipows,Hdxicoeffs] = decomp_ordered(Hdxi,vars);
  dHdecomp{i} = struct('msspoly',Hdxi,'pows',Hdxipows,'coeffs',Hdxicoeffs,'vars',vars);
end
end