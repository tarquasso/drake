%% Set the following parameters to false after running it for the first time:
%clear all
close all

%% Parse in Optitrack Capture Data
%loaded preprocessed data set

if(true)
  dataSetName = 'set5';
  name = '16-May-2015 20_46_41';
  filename = ['~/soft_modeling_repo/dev/tracking/data/',dataSetName,'/',name];
  capturedDataFlag = true;
else
  dataSetName = 'set2';
  name = 'syntheticPaddleDataWithAccel';
  filename = ['~/soft_modeling_repo/dev/simulation/',dataSetName,'/',name];
  capturedDataFlag = false;
end

appendix = '_preprocessed';
fullFilename = [filename,appendix,'.mat'];
load(fullFilename)
[pathstr,name,ext] = fileparts(fullFilename);

%% flags that define what part to execute
calcBSurfaceFlag = false;
calcOneDofProblem = false; % calculating one dof problem
calcThetaFlag = false;
generateDataPointsInBetween = false;
parameterEstimationStep = false;

estimateParamsFminConFlag = false; %using fmin con to estimate the parameters for each contact phase individually
calcAllOfOneCombinedFlag = true; % using fmincon to estimate the parameters for all contact phases as one data set

% timeStepsNC,zNC,zdNC,zddNC,timeStepsLargeNC,zLargeNC,zdLargeNC,zddLargeNC,zfitNC


% generatePlot = true;

%% Estimate bsurface

if(calcBSurfaceFlag)
  
  numOfSetsNC = size(zdNC,1);
  bsurfaceEstViscous = zeros(numOfSetsNC,2);
  force = cell(numOfSetsNC,1);
  forcehat = cell(numOfSetsNC,1);
  r = cell(numOfSetsNC,1);
  bsurfaceEst = zeros(numOfSetsNC,3);
  
  rangeTested = 1:numOfSetsNC;
  
  figure(30);clf;
  
  figure(40);clf;hold on;
  set(gca,'FontSize',plotFontSize)
  xhandle=get(gca,'Xlabel');
  yhandle=get(gca,'Ylabel');
  set(xhandle,'Fontsize',xHandleFontSize)
  set(yhandle,'Fontsize',yHandleFontSize)
  title('Force Estimate Individual')
  xlabel('Time $t \ [s]$','Interpreter','LaTex')
  ylabel('Force $F \ [N]$','Interpreter','LaTex')
  
  figure(41);clf;hold on;
  title('residuals')
  
  for j = rangeTested
    t = timeStepsNC{j};
    q = [zNC{j}'];
    qd = [zdNC{j}'];
    qdd = [zddNC{j}'];
    [bsurfaceEst(j,:),force{j},forcehat{j},r{j},~] = ordinaryLeastSquaresNoContact(qd, qdd, angleDeg, mdisc);
    
    %bsurfaceEstTustin(j,:) = ordinaryLeastSquaresNoContactTustin(qd, qdd, angleDeg, mdisc);
    
    figure(30);
    subplot(3,1,1)
    
    plot(t,q,'k-..');
    hold on;
    %plot(t,q,'k');
    %figure(31);
    subplot(3,1,2)
    plot(t,qd,'k-..');
    hold on;
    %plot(t,qd,'k');
    %figure(32);
    subplot(3,1,3)
    plot(t,qdd,'k-..');
    hold on;
    %plot(t,qdd,'k');
    figure(40)
    plot(t,force{j},'.-.r');
    plot(t,forcehat{j},'-*k'); %,t,forcehat{j},'r*');
    h = legend('Data','Estimate','Location','NorthWest');
    set(h,'Interpreter','Latex');
    figure(41)
    plot(t,r{j},'-..');
    %plot(t,r{j});
    
  end
  
  figure(30);
  subplot(3,1,1)
  axis([-inf inf zTouch inf])
  title('zNC')
  subplot(3,1,2)
  axis([-inf inf -inf inf])
  
  title('zdNC')
  subplot(3,1,3)
  title('zddNC')
  axis([-inf inf -inf inf])
  
  typeofPlot = 'bsurface_allzNC';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(40);    
  typeofPlot = 'bsurface_force';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(41);
  title('residuals')
  
  typeofPlot = 'bsurface_residuals';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  plotFontSize = 17;
  xHandleFontSize = 17;
  yHandleFontSize = 17;
  
  figure(34); clf; hold on;
  i = 1;
  ylabel(['$b_{f,',num2str(i-1),'}$'],'Interpreter','LaTex');
  xlabel('Data Set Number');
  title(['Surface Friction ','$b_{f,',num2str(i-1),'}$'],'Interpreter','LaTex');

  %axis([-inf inf 0.0 1])
  
  set(gca,'FontSize',plotFontSize)
  xhandle=get(gca,'Xlabel');
  yhandle=get(gca,'Ylabel');
  set(xhandle,'Fontsize',xHandleFontSize)
  set(yhandle,'Fontsize',yHandleFontSize)
  
  for i = 1:1
      bSlip = bsurfaceEst(rangeTested,i);
      bSlipMean = mean(bSlip);
      
      figure(33+i);
      %plot(rangeTested,bSlip);
      plot(rangeTested,bSlip,'.-.k','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor','r','markers',22);
      plot(rangeTested([1,end]),[bSlipMean,bSlipMean],'LineWidth',2);
    legend('Individual Phases','Mean','Location','NorthWest');

  end
  %% Estimate All Data
  figure(34)

  typeofPlot = 'bsurface';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  
  heightThreshold = 0.05+zTouch;
  heightThreshold = inf;
  
  
  tBatch = [];
  qBatch = [];
  qdBatch = [];
  qddBatch = [];
  
  rangeTestedBatch = 1:numOfSetsNC;
  
  for j =rangeTestedBatch
    
    idx = find (zNC{j}<heightThreshold);
    tBatchNew = (timeStepsNC{j}(idx))';
    tBatch = [tBatch,tBatchNew];
    
    qNew = (zNC{j}(idx))';
    qBatch = [qBatch,qNew];
    
    qdNew = (zdNC{j}(idx))';
    qdBatch = [qdBatch,qdNew];
    
    qddNew = (zddNC{j}(idx))';
    qddBatch = [qddBatch,qddNew];
  end
  
  [bSurfaceEstBatch,frictionForceBatch,frictionForcehatBatch,residualsBatch,~] = ordinaryLeastSquaresNoContact(qdBatch, qddBatch, angleDeg, mdisc);
  
  bfrictionSurface = bSurfaceEstBatch(1)
  
  figure(50);clf; hold on;
  plot(tBatch,qBatch,'k');
  title('zBatch')
  figure(51);clf; hold on;
  plot(tBatch,qdBatch,'k');
  title('zdBatch')
  figure(52);clf; hold on;
  plot(tBatch,qddBatch,'k');
  title('zddBatch')
  
  figure(53);clf; hold on;
  set(gca,'FontSize',plotFontSize)
  xhandle=get(gca,'Xlabel');
  yhandle=get(gca,'Ylabel');
  set(xhandle,'Fontsize',xHandleFontSize)
  set(yhandle,'Fontsize',yHandleFontSize)
  
  plot(tBatch,frictionForceBatch,'.r','markers',12);
  plot(tBatch,frictionForcehatBatch,'*k','markers',8);
  legend('Data','Estimated','Location','NorthWest');
  title('Force Estimate Batch')
  xlabel('Time $t \ [s]$','Interpreter','LaTex')
  ylabel('Force $F \ [N]$','Interpreter','LaTex')
  
    typeofPlot = 'bsurface_force_batch';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  figure(54); hold on  
  set(gca,'FontSize',plotFontSize)
  xhandle=get(gca,'Xlabel');
  yhandle=get(gca,'Ylabel');
  set(xhandle,'Fontsize',xHandleFontSize)
  set(yhandle,'Fontsize',yHandleFontSize)
  
  plot(tBatch,residualsBatch,'.k');
  title('residualsBatch')
  xlabel('Time $t \ [s]$','Interpreter','LaTex')
  ylabel('residuals','Interpreter','LaTex')
  

  %% Test out the estimate
  for j = rangeTestedBatch
    
    t0 = timeStepsNC{j}(1);
    tf = timeStepsNC{j}(end);
    z0 = zNC{j}(1);
    zd0= zdNC{j}(1);
    m = mdisc;
    %b = bsurfaceEst(j,1);
    bsurface = bSurfaceEstBatch(1); %this estimate is used in the end
    g = 9.81;
    
    % syms z(t)
    % Dz = diff(z);
    % z(t) = dsolve(diff(z, t, t) == - b/m * Dz - g*sind(angleDeg), z(t0) == z0, Dz(t0) == zd0);
    % z(t) = simplify(z)
    
    %mDisc*z'' + 0.1728*z'  + 0.01043*sign(z') = 0.131*9.81*sin(36/180*pi), z(0) = 0.05309, z'(0)=1.668
    
    zdA = @(t) (m*g/bsurface *sind(angleDeg)*(exp(-bsurface/m*(t-t0))-1) + zd0 * exp(-bsurface/m*(t-t0)));
    figure(31)
    tA = linspace(t0,tf,100);
    
    zdAEval = zdA(tA);
    
    plot(tA,zdAEval,'r','LineWidth',1)
    
  end
  
  appendix = '_bSurface';
  fullFilenameBSurface = [filename,appendix,'.mat'];
  
  save(fullFilenameBSurface, 'bsurface');
else
  appendix = '_bSurface';
  fullFilenameBSurface = [filename,appendix,'.mat'];
  
  load(fullFilenameBSurface);
end

%return


numOfSets = size(zIC,1);


%% 1DOF Estimates
if(calcOneDofProblem)
  
  %% 1DOF Estimates
  
  tICBatch1d = [];
  zICBatch1d = [];
  zdICBatch1d = zICBatch1d;
  zddICBatch1d = zICBatch1d;
  rangeOfSets1DOF = 1:numOfSets;
  %rangeOfSets1DOF = 2:2; %picking contact phase 2
  
  numOfSetsAdj1DOF = length(rangeOfSets1DOF);
  
  figure(101); clf; hold on;
  
  figure(201); clf; hold on;
  figure(202); clf; hold on;
  
  for j = rangeOfSets1DOF
    tICBatch1d = [tICBatch1d,timeStepsIC{j}'];
    %IMPORTANT: Adjust for Height offset
    zICAdj{j} = zIC{j}-zTouch;
    zICBatch1d = [zICBatch1d,zICAdj{j}'];
    zdICBatch1d = [zdICBatch1d,zdIC{j}'];
    zddICBatch1d = [zddICBatch1d,zddIC{j}'];
    figure(101);
    subplot(3,1,1)
    
    plot(timeStepsIC{j}',zICAdj{j}','-.ob')
    hold on
    ylabel('z')
    xlabel('Time $t \ [s]$','Interpreter','LaTex')
    title('Z 1-D Model')
    
    subplot(3,1,2)
    plot(timeStepsIC{j}',zdIC{j}','-.ob')
    hold on
    ylabel('zd')
    xlabel('Time $t \ [s]$','Interpreter','LaTex')
    title('ZD 1-D Model')
    
    subplot(3,1,3)
    plot(timeStepsIC{j}',zddIC{j}','-.ob')
    hold on
    ylabel('zdd')
    xlabel('Time $t \ [s]$','Interpreter','LaTex')
    title('ZDD 1-D Model')
    
    
    rangeOfModelComplexities1d = 0:0; % just first order model b = (b0 + b1*deltaz)
    numOfModels1d = length(rangeOfModelComplexities1d);
    for k = 1:numOfModels1d
      l = rangeOfModelComplexities1d(k);
      Mb = l;
      Mk = l;
      [gammaSingle1d{j},WSingle1d{j}] = softContactModel1D(zICAdj{j}', zdIC{j}', zddIC{j}', ...
        angleDeg, mdisc, bsurface, Mb, Mk);
      alpha1d = 0.05; % 95% confidence level
      
      [bSingle1d{j,k},bintSingle1d{j,k},rSingle1d{j,k},rintSingle1d{j,k}] = regress(gammaSingle1d{j},WSingle1d{j},alpha1d); %,statsSingle1d(k,:)
      %mdlSingle1d = fitlm(WSingle1d{j},gammaSingle1d{j});%,'Intercept',false) %model fit without intercept term
      %figure
      %plotResiduals(mdl1d)
      %mdl1dstep = step(mdl1d,'NSteps',20)
      %figure
      %plotResiduals(mdl1dstep)
      
    end
    
    b0Index = 1;
    b0Est = bSingle1d{j,k}(b0Index);
    b0EstErrLow = bSingle1d{j,k}(b0Index)-bintSingle1d{j,k}(b0Index,1);
    b0EstErrUp = bintSingle1d{j,k}(b0Index,2)-bSingle1d{j,k}(b0Index);
    
    k0Index = 1+l+1;
    k0Est = bSingle1d{j,k}(k0Index);
    k0EstErrLow = bSingle1d{j,k}(k0Index)-bintSingle1d{j,k}(k0Index,1);
    k0EstErrUp = bintSingle1d{j,k}(k0Index,2)-bSingle1d{j,k}(k0Index);
    
    figure(201);
    errorbar(j,b0Est,b0EstErrLow,b0EstErrUp,...
      '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',18);
    hold on;
    xlabel('data set number');title('b0 of 1D Model');
    
    figure(202);
    errorbar(j,k0Est,k0EstErrLow,k0EstErrUp,...
      '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',18);
    hold on;
    xlabel('data set number');title('k0 of 1D Model');
    
  end
  
  %% Batch Evaluation
  
  % Try different model complexities
  
  rangeOfModelComplexities1d = 0:0; % just first order model b = (b0 + b1*deltaz)
  numOfModels1d = length(rangeOfModelComplexities1d);
  for k = 1:numOfModels1d
    l = rangeOfModelComplexities1d(k);
    Mb = l;
    Mk = l;
    [gammaBatch1d,WBatch1d] = softContactModel1D(zICBatch1d, zdICBatch1d, zddICBatch1d, ...
      angleDeg, mdisc, bsurface, Mb, Mk);
    alpha1d = 0.05; % 95% confidence level
    
    [bBatch1d{k},bintBatch1d{k},rBatch1d{k},rintBatch1d{k}] = regress(gammaBatch1d,WBatch1d,alpha1d); %,statsBatch1d(k,:)
    mdl1d = fitlm(WBatch1d,gammaBatch1d);%,'Intercept',false) %model fit without intercept term
    %figure
    %plotResiduals(mdl1d)
    %mdl1dstep = step(mdl1d,'NSteps',20)
    %figure
    %plotResiduals(mdl1dstep)
    
    b0Index = 1;
    b0Est = bBatch1d{k}(b0Index);
    b0EstErrLow = bintBatch1d{k}(b0Index,1);
    b0EstErrUp = bintBatch1d{k}(b0Index,2);
    
    k0Index = 1+l+1;
    k0Est = bBatch1d{k}(k0Index);
    k0EstErrLow = bintBatch1d{k}(k0Index,1);
    k0EstErrUp = bintBatch1d{k}(k0Index,2); %absolute value
    
    figure(201);
    plot(rangeOfSets1DOF,ones(1,numOfSetsAdj1DOF)*b0Est);
    plot(rangeOfSets1DOF,ones(1,numOfSetsAdj1DOF)*b0EstErrUp,'--g');
    plot(rangeOfSets1DOF,ones(1,numOfSetsAdj1DOF)*b0EstErrLow,'--m');
    
    figure(202);
    plot(rangeOfSets1DOF,ones(1,numOfSetsAdj1DOF)*k0Est);
    plot(rangeOfSets1DOF,ones(1,numOfSetsAdj1DOF)*k0EstErrUp,'--g');
    plot(rangeOfSets1DOF,ones(1,numOfSetsAdj1DOF)*k0EstErrLow,'--m');
    
  end
  
  figure(201);
  
  typeofPlot = 'b01d';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  figure(202);
  
  typeofPlot = 'k01d';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  %%   Simulate the system
  for j = rangeOfSets1DOF
    
    %gravity
    gravity = 9.81;
    gStar = gravity*sind(angleDeg);
    params = bBatch1d{1};
    bStar = params(1:k0Index-1)%+bsurface;
    kStar = params(k0Index:k0Index+l)
    mStar =  mdisc
    tfinal = timeStepsIC{j}(end)-timeStepsIC{j}(1);
    z0 = zICAdj{j}(1);
    zd0 = zdIC{j}(1);
    
    sol = simulateSecondOrderSystem(gStar,bStar,kStar, Mb, Mk,mStar,tfinal, z0,zd0);
    
    figure(101);
    timeSteps = timeStepsIC{j}-timeStepsIC{j}(1); %linspace(0,tf,10);
    [y,yd] = deval(sol,timeSteps);
    
    subplot(3,1,1)
    plot(timeStepsIC{j},y(1,:),'-.or')
    legend('data','estimated','Location','SouthEast')
    subplot(3,1,2)
    plot(timeStepsIC{j},y(2,:),'-.or')
    legend('data','estimated','Location','SouthEast')
    
    subplot(3,1,3)
    plot(timeStepsIC{j},yd(2,:),'-.or')
    legend('data','estimated')
    
    %fplot(@(x)deval(sol,x,1), timeRange)
    
  end
  
  figure(101)
  typeofPlot = 'z1dof';
  options.Format = 'eps';
  hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
  
  
  %   figure(601); clf; hold on;
  %   plot(rangeOfModelComplexities1d,statsBatch1d(:,1));
  %   xlabel('model');title('Rsq statistics 1d');
  %
  %   figure(602); clf; hold on;
  %   plot(rangeOfModelComplexities1d,statsBatch1d(:,2));
  %   xlabel('model');title('F stat 1d');
  %
  %   figure(603); clf; hold on;
  %   plot(rangeOfModelComplexities1d,statsBatch1d(:,3));
  %   xlabel('model');title('p value 1d');
  %
  %   figure(604); clf; hold on;
  %   plot(rangeOfModelComplexities1d,statsBatch1d(:,4));
  %   xlabel('model');title('error covariance 1d');
  
end

%% Estimate Theta
if(calcThetaFlag)
  thetaIC0 = 0;
  thetadIC0 = 0;
  
  optionsfmincon = optimoptions(@fmincon);
  optionsfmincon = optimoptions(optionsfmincon, 'SpecifyObjectiveGradient', true, 'Display', 'none');
  lb = [-pi;-100];
  ub = [pi;100];
  
  thetaOrigIC = cell(numOfSets,1);
  %thetadfmincon = cell(numOfSets,1);
  %thetaVec = zeros(size(z{1},1),2);
  %theta = zeros(size(z{1},1),1);
  
  for j = 1:numOfSets
    numOfSamples = size(zIC{j},1);
    for k =1:numOfSamples
      %fun = @(thetaVec) resolveConstraintThetaThetaDotCostFun([thetaVec(1);0;z{j}(k)], [thetaVec(2);0;zd{j}(k)]);
      %[thetaVecTemp,fval] = fmincon(fun, [theta0; thetad0], [], [], [], [], lb, ub, [], options);
      %thetadfmincon{j}(k,1) = thetaVecTemp(2);
      fun = @(theta) resolveConstraintThetaCostFun([theta;0;zIC{j}(k)]);
      [thetaTemp,fval] = fmincon(fun, thetaIC0, [], [], [], [], lb(1), ub(1), [], optionsfmincon);
      
      thetaOrigIC{j}(k,1) = thetaTemp;
      thetaIC0 = thetaTemp;
      %theta0 = thetaVecTemp(1);
      %thetad0 = thetaVecTemp(2);
      
    end
  end
  
  %% fit curves for theta
  thetafitIC = cell( numOfSets, 1 );
  thetaIC = cell( numOfSets, 1 );
  thetadIC = cell( numOfSets, 1 );
  thetaddIC = cell( numOfSets, 1 );
  
  gofTheta = struct( 'sse', cell( numOfSets, 1 ), ...
    'rsquare', [], 'dfe', [], 'adjrsquare', [], 'rmse', [] );
  ft = fittype( 'poly5' );
  
  figure(500); clf; hold on;
  figure(501); clf; hold on;
  for j = 1: numOfSets
    
    figure(500);  hold on; plot(timeStepsIC{j}, thetaOrigIC{j},'b*');
    [tData, thetaData] = prepareCurveData(  timeStepsIC{j}, thetaOrigIC{j});
    
    plot(tData, thetaData,'r+');
    [thetafitIC{j}, gofTheta(j)] = fit( tData, thetaData, ft ,'Normalize','on');
    
    thetaIC{j} = feval(thetafitIC{j},timeStepsIC{j}); %take fitted theta values
    
    [thetadIC{j}, thetaddIC{j}] = differentiate(thetafitIC{j},timeStepsIC{j});
    
    figure(501);
    p = plot(thetafitIC{j},tData,thetaData);%,[timeInterval{j}])
    p(1).LineWidth = 2;
  end
  
  
  appendix = '_theta';
  fullFilenameTheta = [filename,appendix,'.mat'];
  
  save(fullFilenameTheta, 'timeStepsIC', 'zIC','zdIC','zddIC','thetaOrigIC','thetaIC','thetadIC','thetaddIC','thetafitIC');%,'thetadfmincon');
else
  appendix = '_theta';
  fullFilenameTheta = [filename,appendix,'.mat'];
  if(capturedDataFlag)
    load(fullFilenameTheta);
  end
end

thetaNC = zNC;
for k=1:length(thetaNC)
thetaNC{k} = 0*thetaNC{k};
end
thetadNC = thetaNC;
thetaddNC = thetaNC;



figure(502); clf;
subplot(3,1,1); hold on
set(gca,'FontSize',plotFontSize)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSize)
set(yhandle,'Fontsize',yHandleFontSize)
set(gca,'XTick',[]);
%xlabel('Time $t \ [s]$','Interpreter','LaTex')
ylabel('$\theta \ [rad]$','Interpreter','LaTex')
title('Virtual Pulley Angle and Derivatives ($\theta, \dot{\theta} , \ddot{\theta}$)','Interpreter','Latex');
axis([timeStepsNC{1}(1) timeStepsIC{end}(end) min(thetaIC{1})*1.01 max(thetaNC{1})*1.01])

subplot(3,1,2); hold on
set(gca,'FontSize',plotFontSize)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSize)
set(yhandle,'Fontsize',yHandleFontSize)
set(gca,'XTick',[]);
%xlabel('Time $t \ [s]$','Interpreter','LaTex')
ylabel('$\dot{\theta} \ [\frac{rad}{s}]$','Interpreter','LaTex')
%title('Angular Velocity of the Virtual Pulley $\dot{\theta}$','Interpreter','Latex');
axis([timeStepsNC{1}(1) timeStepsIC{end}(end) min(thetadIC{1})*1.01 max(thetadIC{1})*1.01])

subplot(3,1,3); hold on;
set(gca,'FontSize',plotFontSize)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSize)
set(yhandle,'Fontsize',yHandleFontSize)
xlabel('Time $t \ [s]$','Interpreter','LaTex')
ylabel('$\ddot{\theta} \ [\frac{rad}{s^2}]$','Interpreter','LaTex')
%title('Angular Acceleration of the Virtual Pulley $\ddot{\theta}$','Interpreter','Latex');
axis([timeStepsNC{1}(1) timeStepsIC{end}(end) min(thetaddIC{1})*1.01 max(thetaddIC{1})*1.01])

plotFontSizeWide = 30;
xHandleFontSizeWide = 30;
yHandleFontSizeWide = 30;
markerSizeICWide = 30;
markerSizeNCWide = 15;
lineWidthSizeWide = 3.5;

sizeDescriptor = [.1 .1 1 .4];
hFig1 = figure(503); clf; hold on
set(hFig1,'units','normalized','position',sizeDescriptor);

set(gca,'FontSize',plotFontSizeWide)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSizeWide)
set(yhandle,'Fontsize',yHandleFontSizeWide)
xlabel('Time $t \ [s]$','Interpreter','LaTex')
ylabel('$\theta \ [rad]$','Interpreter','LaTex')
% title('Virtual Pulley Angle and Derivatives ($\theta, \dot{\theta} , \ddot{\theta}$)','Interpreter','Latex');
axis([timeStepsNC{1}(1) timeStepsIC{end}(end) min(thetaIC{1})*1.01 max(thetaNC{1})*1.01])

hFig2 = figure(504); clf; hold on
set(hFig2,'units','normalized','position',sizeDescriptor);

set(gca,'FontSize',plotFontSizeWide)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSizeWide)
set(yhandle,'Fontsize',yHandleFontSizeWide)
xlabel('Time $t \ [s]$','Interpreter','LaTex')
ylabel('$\dot{\theta} \ [\frac{rad}{s}]$','Interpreter','LaTex')
%title('Angular Velocity of the Virtual Pulley $\dot{\theta}$','Interpreter','Latex');
axis([timeStepsNC{1}(1) timeStepsIC{end}(end) min(thetadIC{1})*1.01 max(thetadIC{1})*1.01])

hFig3 =figure(505); clf; hold on
set(hFig3,'units','normalized','position',sizeDescriptor);

set(gca,'FontSize',plotFontSizeWide)
xhandle=get(gca,'Xlabel');
yhandle=get(gca,'Ylabel');
set(xhandle,'Fontsize',xHandleFontSizeWide)
set(yhandle,'Fontsize',yHandleFontSizeWide)
xlabel('Time $t \ [s]$','Interpreter','LaTex')
ylabel('$\ddot{\theta} \ [\frac{rad}{s^2}]$','Interpreter','LaTex')
%title('Angular Acceleration of the Virtual Pulley $\ddot{\theta}$','Interpreter','Latex');
axis([timeStepsNC{1}(1) timeStepsIC{end}(end) min(thetaddIC{1})*1.01 max(thetaddIC{1})*1.01])


for j = 1: numOfSets
  
  %figure(500);
  %plot(timeStepsIC{j}, thetaOrigIC{j},'b*');
  %p = plot(thetafitIC{j},timeStepsIC{j},thetaIC{j});%,[timeInterval{j}])
  %p(1).LineWidth = 2;
  
  figure(502);
  subplot(3,1,1)
  plot(timeStepsIC{j},thetaIC{j},'-..r','markers',9)
  plot(timeStepsNC{j},thetaNC{j},'-..b','markers',5)

  subplot(3,1,2)
  plot(timeStepsIC{j},thetadIC{j},'-..r','markers',9)
  plot(timeStepsNC{j},thetadNC{j},'-..b','markers',5)
  
  subplot(3,1,3)
  plot(timeStepsIC{j},thetaddIC{j},'-..r','markers',9)
  plot(timeStepsNC{j},thetaddNC{j},'-..b','markers',5)
  
  
  figure(503);
  plot(timeStepsIC{j},thetaIC{j},'-..r','LineWidth',lineWidthSizeWide,'markers',markerSizeICWide)
  plot(timeStepsNC{j},thetaNC{j},'-..b','LineWidth',lineWidthSizeWide,'markers',markerSizeNCWide)
  
  figure(504);
  plot(timeStepsIC{j},thetadIC{j},'-..r','LineWidth',lineWidthSizeWide,'markers',markerSizeICWide)
  plot(timeStepsNC{j},thetadNC{j},'-..b','LineWidth',lineWidthSizeWide,'markers',markerSizeNCWide)

  figure(505);
  plot(timeStepsIC{j},thetaddIC{j},'-..r','LineWidth',lineWidthSizeWide,'markers',markerSizeICWide)
  plot(timeStepsNC{j},thetaddNC{j},'-..b','LineWidth',lineWidthSizeWide,'markers',markerSizeNCWide)
  
end
 


figure(502);
typeofPlot = 'alltheta';
optionsPlot.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],optionsPlot);

figure(503);
typeofPlot = 'theta';
optionsPlot.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],optionsPlot);

figure(504);
typeofPlot = 'thetad';
optionsPlot.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],optionsPlot);

figure(505);
typeofPlot = 'thetadd';
optionsPlot.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],optionsPlot);

%% Generate MORE DATA points in between using fit

if(generateDataPointsInBetween)
  timeSteps2 = cell( numOfSets, 1 );
  z2 = cell( numOfSets, 1 );
  zd2 = cell( numOfSets, 1 );
  zdd2 = cell( numOfSets, 1 );
  theta2 = cell( numOfSets, 1 );
  thetad2 = cell( numOfSets, 1 );
  thetadd2 = cell( numOfSets, 1 );
  numOfLargeData = 500;
  
  for j=1:numOfSets
    timeSteps2{j} = linspace(timeStepsIC{j}(2),timeStepsIC{j}(end-1),numOfLargeData);
    z2{j} = feval(zfitIC{j},timeSteps2{j});
    [zd2{j}, zdd2{j}] = differentiate(zfitIC{j},timeSteps2{j});
    theta2{j} = feval(thetafitIC{j},timeSteps2{j});
    [thetad2{j}, thetadd2{j}] = differentiate(thetafitIC{j},timeSteps2{j});
  end
end

%% Initial Parameter Guesses
if(parameterEstimationStep)

%for dim =3
% q = [theta{j}';...
%     zeros(size(z{j}))';...
%     z{j}'];
% qd = [thetad{j}';...
%     zeros(size(zd{j}))';...
%     zd{j}'];
% qdd = [thetadd{j}';...
%     zeros(size(zdd{j}))';...
%     zdd{j}'];

%for dim =2

rangeOfSets = 1:numOfSets;
%rangeOfSets = 3:3;

numOfSetsAdj = length(rangeOfSets);

pEst = cell(numOfSetsAdj,1);

for i =1:numOfSetsAdj
  
  j =  rangeOfSets(i);
  
  if(generateDataPointsInBetween)
    q = [theta2{j}';...
      z2{j}'];
    qd = [thetad2{j}';...
      zd2{j}'];
    qdd = [thetadd2{j}';...
      zdd2{j}'];
  else
    q = [thetaIC{j}';...
      zIC{j}'];
    qd = [thetadIC{j}';...
      zdIC{j}'];
    qdd = [thetaddIC{j}';...
      zddIC{j}'];
  end
  if(estimateParamsFminConFlag)
    min = 0.00000001;
    max = 100;
    Ipulley = 0.00001798;
    kpulley = 0.22;
    bpulley = 0.0024;
    % note: mdisc is not a parameter to be estimated
    
    b0 = [Ipulley, kpulley, bpulley]; % simulation hand tuning
    dimParams = size(b0,2);
    
    fun = @(p) paramEstCostFun2D(p, q, qd, qdd, angleDeg, mdisc, bsurface);
    optionsfmincon = optimoptions(@fmincon);
    optionsfmincon = optimoptions(optionsfmincon, 'Display', 'iter');
    [pEst{i},fval] = fmincon(fun, b0, [],[], [], [], min*ones(1,dimParams), max*ones(1,dimParams),[], optionsfmincon);
    save('params.mat', 'pEstimated');
  else
    rangeOfModels = 0:0;
    numOfModels = length(rangeOfModels);
    for j = 1:numOfModels
      l = rangeOfModels(j);
      Mb = l;
      Mk = l;
      [gamma,W] = softContactModel2D(q, qd, qdd, angleDeg, mdisc, bsurface, Mb, Mk);
      alpha = 0.05; % 95% confidence level
      [b{i,j},bint{i,j},r{i,j},rint{i,j}] = regress(gamma,W,alpha);
      %load('params.mat');
    end
  end
  
end
%

I0Est= zeros(numOfSetsAdj,1);
IpulleyEstErr= zeros(numOfSetsAdj,2);

k0Est= zeros(numOfSetsAdj,1);
kpulleyEstErr= zeros(numOfSetsAdj,2);

b0Est= zeros(numOfSetsAdj,1);
bpulleyEstErr= zeros(numOfSetsAdj,2);


for i=1:numOfSetsAdj
  for j = 1:numOfModels
    l = rangeOfModels(j);
    I0Index = 1;
    I0Est(i,j) = b{i,j}(I0Index);
    I0EstErrLow(i,j) = bint{i,j}(I0Index,1)-b{i,j}(I0Index);
    I0EstErrUp(i,j) = bint{i,j}(I0Index,2)-b{i,j}(I0Index);
    
    b0Index = 1+1;
    b0Est(i,j) = b{i,j}(b0Index);
    b0EstErrLow(i,j) = b{i,j}(b0Index)-bint{i,j}(b0Index,1);
    b0EstErrUp(i,j) = bint{i,j}(b0Index,2)-b{i,j}(b0Index);
    
    k0Index = 1+l+1+1;
    k0Est(i,j) = b{i,j}(k0Index);
    k0EstErrLow(i,j) = b{i,j}(k0Index)-bint{i,j}(k0Index,1);
    k0EstErrUp(i,j) = bint{i,j}(k0Index,2)-b{i,j}(k0Index);
    
  end
end

rangeOfSetsMat = repmat(rangeOfSets',1,numOfModels);
rangeOfModelsCell = strcat({'Model '},int2str(rangeOfModels.')).';

figure(701); clf; hold on;
errorbar(rangeOfSetsMat,I0Est,I0EstErrLow,I0EstErrUp,...
  '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',18);
xlabel('data set number');title('I0');
legend(rangeOfModelsCell)
typeofPlot = 'I0Est';
options.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);

figure(702); clf; hold on;
errorbar(rangeOfSetsMat,b0Est,b0EstErrLow,b0EstErrUp,...
  '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',18);
xlabel('data set number');title('b0');
legend(rangeOfModelsCell)
typeofPlot = 'b0Est';
options.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);

figure(703); clf; hold on;
errorbar(rangeOfSetsMat,k0Est,k0EstErrLow,k0EstErrUp,...
  '-s','MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red','CapSize',18);
xlabel('data set number');title('k0');
legend(rangeOfModelsCell)
typeofPlot = 'k0Est';
options.Format = 'eps';
hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);

%% Batch Estimate
if(calcAllOfOneCombinedFlag)
  tBatch = [];
  qBatch = [];
  qdBatch = qBatch;
  qddBatch = qBatch;
  
  for j =rangeOfSets
    tBatch = [tBatch,timeStepsIC{j}'];
    qNewBatch = [thetaIC{j}';...
      zIC{j}'];
    qBatch = [qBatch,qNewBatch];
    
    qdNewBatch = [thetadIC{j}';...
      zdIC{j}'];
    qdBatch = [qdBatch,qdNewBatch];
    
    qddNewBatch = [thetaddIC{j}';...
      zddIC{j}'];
    qddBatch = [qddBatch,qddNewBatch];
    
  end
  
  if(estimateParamsFminConFlag)
    
    %FMINCON
    fun = @(b) paramEstCostFun2D(b, qBatch, qdBatch, qddBatch, angleDeg, mdisc, bsurface);
    optionsfmincon = optimoptions(@fmincon);
    optionsfmincon = optimoptions(optionsfmincon, 'Display', 'iter');
    
    [pEstimatedAll,fval] = fmincon(fun, b0, [],[], [], [], ...
      min*ones(1,dimParams), max*ones(1,dimParams),[], optionsfmincon);
  else
    rangeOfModels = 0:0;
    numOfModels = length(rangeOfModels);
    for j = 1:numOfModels
      l = rangeOfModels(j);
      Mb = l;
      Mk = l;
      [gammaBatch,WBatch] = softContactModel2D(qBatch, qdBatch, qddBatch, ...
        angleDeg, mdisc, bsurface,Mb,Mk);
      alpha = 0.05; % 95% confidence level
      
      [bBatch{j},bintBatch{j},rBatch{j},rintBatch{j}] = regress(gammaBatch,WBatch,alpha); %statsBatch{j}
      mdl = fitlm(WBatch,gammaBatch);%,'Intercept',false) %model fit without intercept term
      %figure
      %plotResiduals(mdl)
      %mdl1 = step(mdl,'NSteps',20)
      %figure
      %plotResiduals(mdl1)
      
      
    end
    
    paramsEstimated.I = bBatch{j}(1);
    paramsEstimated.b = bBatch{j}(2);
    paramsEstimated.bSurface = bfrictionSurface;
    paramsEstimated.k = bBatch{j}(3);
    paramsEstimated.lamdas = bBatch{j}(4:end)';
    figure(1000)
    plot(tBatch,paramsEstimated.lamdas,'-.o')
    title('Lambdas')
    xlabel('time');
    ylabel('force lambda');
    
    typeofPlot = 'lambdas';
    options.Format = 'eps';
    hgexport(gcf,[pathstr,'/plots/',dataSetName,'_',typeofPlot,'.eps'],options);
    %     figure(801); clf; hold on;
    %     plot(rangeOfModels,statsBatch{j}(:,1));
    %     xlabel('model');title('Rsq statistics');
    %
    %     figure(802); clf; hold on;
    %     plot(rangeOfModels,statsBatch{j}(:,2));
    %     xlabel('model');title('F stat');
    %
    %     figure(803); clf; hold on;
    %     plot(rangeOfModels,statsBatch{j}(:,3));
    %     xlabel('model');title('p value');
    %
    %     figure(804); clf; hold on;
    %     plot(rangeOfModels,statsBatch{j}(:,4));
    %     xlabel('model');title('error covariance');
    if(~capturedDataFlag)
      paramsUsed
    end
    paramsEstimated
  end
  
  
  
end
end