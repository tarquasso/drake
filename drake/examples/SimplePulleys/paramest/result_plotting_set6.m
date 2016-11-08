close all

%Set 6
load('~/soft_modeling_repo/dev/tracking/data/set6/16-May-2015 20_53_54.mat')

%number of samples
nos = History.i-1;

% Touching point: 55.8mm
tp = 0.0558;
%Static spring stretching point: 52.8 mm

%%
timeVals =History.timestamps(1:nos);
xVals = History.objectPosition(1:nos,1);
under = find(xVals<tp);
yVals = History.objectPosition(1:nos,2);
zVals = History.objectPosition(1:nos,3);
figure
plot(timeVals,xVals,'b')
hold on
plot(timeVals(under),xVals(under),'r*')
plot(timeVals([1,end]),[tp,tp],'g')
xlabel('time [s]')
ylabel('height coordinate z [m]')
title('Set 6 - Z-Coordinate')
% figure
% plot(timeVals,yVals)
% title('y')
% figure
% plot(timeVals,zVals)
% title('z')