function [utraj,xtraj] = swingUpURDF(model)
%% runs trajectory optimization and animates open-loop playback

xG = Point(model.getStateFrame,[pi;0;0;0]);
uG = Point(model.getInputFrame,0);

x0 = zeros(4,1);
xf = double(xG);
tf0 = 4;

N = 21;
prog = DircolTrajectoryOptimization(model,N,[2 6]); %set up direct collocation optimization
prog = prog.addStateConstraint(ConstantConstraint(x0),1);
prog = prog.addStateConstraint(ConstantConstraint(xf),N);
prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));

for attempts=1:10
    tic
    [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
    toc
    if info==1, break; end
end

    function [g,dg] = cost(dt,x,u)
        R = 1;
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Q = diag([10,10,1,1]);
        R = 100;
        g = sum((Q*xerr).*xerr + (R*u).*u,1);
        
        if (nargout>1)
            dgddt = 0;
            dgdx = 2*xerr'*Q;
            dgdu = 2*u'*R;
            dg = [dgddt,dgdx,dgdu];
        end
    end

    function [h,dh] = finalCost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Qf = 100*diag([10,10,1,1]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
            dh = [0, 2*xerr'*Qf];
        end
    end



end
