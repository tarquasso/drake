function f = costFunCumulative(psi, x, plantSim, c)

xd = [-0.045; 4; 0; -3.1321];
N = 5;      % Desired number of bounces
% N = round(psi(end),0);

m = x(1);
q = x(2:5);
qp = x(6:9);

if m == 2
    xplus = x;
else
    psiInitial = psi(1);
    R = [cos(psiInitial), sin(psiInitial); -sin(psiInitial), cos(psiInitial)];
    load_in_paddle = -[3; 0] + R'*([q(3); q(4)]-[-3;3]);
    loadv_in_paddle = R'*[qp(3); qp(4)];
    
    tf = max(roots([-1/2*9.81*cos(psiInitial), loadv_in_paddle(2), load_in_paddle(2)-1]));
    zTouch = q(4) + qp(4)*tf - 1/2*9.81*tf^2;
    zpTouch = qp(4) - 9.81*tf;
    xTouch = q(3) + qp(3)*tf;
    xpTouch = qp(3);
    
    xplus = [2; psiInitial; q(2); xTouch; zTouch; 0; 0; xpTouch; zpTouch];
end


f = 0;
for i = 1:N
    c.psiDes = psi(i);

    output_select(1).system = 1;
    output_select(1).output = plantSim.getOutputFrame();
    output_select(2).system = 2;
    output_select(2).output = c.getOutputFrame();
    
    sys = mimoFeedback(plantSim,c,[],[],[],output_select);


    
    
    [ytraj,~] = simulate(sys,[0 1],xplus);
    tt=getBreaks(ytraj);
    yAll = ytraj.eval(tt);
    
    % Find mode changes
    jumpIdx = find(diff(yAll(1,:)));
%     try
%         if i == 1
%             xplus = yAll(1:9,jumpIdx(3));
%             phi = xd - xplus([3:4,7:8]+1);
%             f = f + dot(phi, phi);
%             xplus = yAll(1:9, min(jumpIdx(3)+10, length(yAll)));
%         else
%             xplus = yAll(1:9,jumpIdx(2));
%             phi = xd - xplus([3:4,7:8]+1);
%             f = f + dot(phi, phi);
%             xplus = yAll(1:9, min(jumpIdx(2)+10, length(yAll)));
%         end
%     catch
%         xplus = [1; 100*ones(8,1)];
%     end
    
    q = yAll(2:5,jumpIdx(1));
    qp = yAll(6:9,jumpIdx(1));
    
    if i < N
        psiCurrent = psi(i+1);
    else
        psiCurrent = 0;
    end
    
    R = [cos(psiCurrent), sin(psiCurrent); -sin(psiCurrent), cos(psiCurrent)];
    load_in_paddle = -[3; 0] + R'*([q(3); q(4)]-[-3;3]);
    loadv_in_paddle = R'*[qp(3); qp(4)];
    
    tf = max(roots([-1/2*9.81*cos(psiCurrent), loadv_in_paddle(2), load_in_paddle(2)-1]));
    zTouch = q(4) + qp(4)*tf - 1/2*9.81*tf^2;
    zpTouch = qp(4) - 9.81*tf;
    xTouch = q(3) + qp(3)*tf;
    xpTouch = qp(3);
    
    xplus = [2; psiCurrent; q(2); xTouch; zTouch-1e-3; 0; 0; xpTouch; zpTouch];

    phi = xd - xplus([3:4,7:8]+1);
    f = f + dot(phi, phi);

    fprintf(['psi = ', mat2str(round(psi,5)), '\n'])
end

% phi = xd - xplus([3:4,7:8]+1);
% phi = N/10*phi;