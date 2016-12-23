function [Gamma,W] = ordinaryLeastSquaresNoContactPreProcess(qd, qdd, angleDeg, mdisc)
%% Position States Vector: qd = [zd_disc] x numsamples
%% Parameter Vector: p = [bstick,bslip]; 
%LSFit
%https://www.mathworks.com/help/curvefit/least-squares-fitting.html

%rows are position states, columns are samples:
[dim,numSamples] = size(qd);

% Error checking: check if position state dimension is proper
assert(dim==1,'q needs to be 1 dimensional, only supporting straight sliding so far!')
assert(size(qd,1)==size(qdd,1),'qd and qdd do not have same row dimension!')
assert(size(qd,2)==size(qdd,2),'qd and qdd do not contain same number of samples!')

%gravity
g = 9.81;
%angle of inclined plane
beta = angleDeg * pi/180; % angle of the sliding platform for the disk

zd = qd(1,:)';
zdd = qdd(1,:)';
      
%X = zd;%[sign(zd),zd];%,zd.^2] ; 
%X = [sign(zd),zd]; 
%X = zd;
W = [zd];%,zd.^2.*sign(zd),zd.^3.*sign(zd)];
 
Gamma = - mdisc * zdd - mdisc * g * sin(beta) ;

end