function p = ordinaryLeastSquaresNoContactTustin(qd, qdd, angleDeg, mdisc)
%% Position States Vector: qd = [zd_disc] x numsamples
%% Parameter Vector: p = [bslip, bstick]; 

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
      
sgnZd = sign(zd);
Wmat = [sgnZd, -abs(zd).^(1/2).*sgnZd] ; 
       
Gamma = - mdisc * ( zdd + g * sin(beta) ) ;
% Solve Least Squares for parameters:
p = inv(Wmat'* Wmat) * Wmat' * Gamma;

%Transpose
p = p';
end