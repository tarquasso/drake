function [gamma,W] = softContactModel1D(z, zd, zdd, angleDeg, mdisc, bsurface, Mb, Mk)
%% Position States Vector: q = [theta;z_disc] x numsamples
%% Parameter Vector: p = [Ipulley;kpulley;bpulley;lambda(i)]; %lambda(i) for each sample 

%rows are position states, columns are samples:
[dim,numSamples] = size(z);

% Error checking: check if position state dimension is proper
assert(dim==1,'q needs to be 1 dimensional!')
assert(size(z,1)==size(zd,1),'q and qd do not have same row dimension!')
assert(size(z,2)==size(zd,2),'q and qd do not contain same number of samples!')
assert(size(zd,1)==size(zdd,1),'qd and qdd do not have same row dimension!')
assert(size(zd,2)==size(zdd,2),'qd and qdd do not contain same number of samples!')


z = z';
zd = zd';
zdd = zdd';

%gravity
g = 9.81;
%angle of inclined plane
beta = angleDeg * pi/180; % angle of the sliding platform for the disk

polyfun = @(base,exponent) base.^exponent;
%calculate the polynomial basis
Phib = bsxfun(polyfun,z,(0:Mb));
Phik = bsxfun(polyfun,z,(0:Mk));

W = [(zdd+g*sin(beta)), zd.*Phib, z.*Phik];%,  oneVec];

gamma = -bsurface* zd;% - mdisc * zdd - mdisc * g * sin(beta);  

% Solve Least Squares for parameters:
%p = inv(Wmat'* Wmat) * Wmat' * Gamma;

%Transpose
%p = p';
end