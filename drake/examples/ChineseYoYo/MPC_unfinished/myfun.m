function f = myfun(kappa, p, plantSim, tf)

c = SoftPaddleControlRealMl(p);

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

c.kappa0 = kappa(1);
c.kappa1 = kappa(2);
c.kappa00 = kappa(3);
c.kappa01 = kappa(4);

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

x0 = p.getInitialState();

xFixed = -0.0025;

try
%     tic
    [ytraj,xtraj] = simulate(sys,[0 tf],x0);
%     toc
catch
    f = 10;
end

t = linspace(xtraj.tspan(1), xtraj.tspan(end), 1001);
x = eval(xtraj, t);
f = 0*(x(4,:)-xFixed)*(x(4,:)-xFixed)' + 0.1*x(8,:)*x(8,:)';

kappaShow = round(kappa, 4);
fprintf(['kappa = ', mat2str(kappaShow), ', f = ', num2str(f), '\n'])

end