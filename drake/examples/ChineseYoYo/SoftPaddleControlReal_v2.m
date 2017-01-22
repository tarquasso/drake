classdef SoftPaddleControlReal_v2 < DrakeSystem
  properties
    kp
    kd
    plant
    K
  end
  
  methods
    function obj = SoftPaddleControlReal_v2(plant)
      %obj = obj@HybridDrakeSystem(2*plant.in_contact.num_positions+1,size(plant.in_contact.B,2));
      obj = obj@DrakeSystem(0,0,9,1,true,true);
      obj = setInputFrame(obj, getOutputFrame(plant));
      obj = setOutputFrame(obj, getInputFrame(plant));
      obj.kp = 5000;
      obj.kd = 2*sqrt(obj.kp);
      obj.plant = plant;
      
      K = [-1 0.0016654 -0.55775];

      obj.K = -K; %TODO: Fix the code to not need the sign inversion
      assignin('base', 'uold', 0);
      assignin('base', 'count', 0);
    end
    
    function u = output(obj,t,~,x)
        m = x(1);
        q = x(2:5);
        qp = x(6:9);
        
        psi = q(1); theta = q(2); x = q(3); z = q(4);
        psip = qp(1); thetap = qp(2); xp = qp(3); zp = qp(4);
        
        zmax = 0.30;
        beta = 35.8;
        ox = -0.271;
        oz = 0;
        r = 0.0439;
        
        % Inverse dynamics
        [H,C,B] = manipulatorDynamics(obj.plant.no_contact, q, qp);
        [phi,J,ddphi] = obj.plant.in_contact.position_constraints{1}.eval(q);
        Jp = reshape(ddphi,length(q),[])*qp;
        Hinv = inv(H);
        Hinvtilde = J*Hinv*J';
%         Delta = Hinvtilde - J(1)*J*Hinv*B;
        e1 = [1; 0; 0; 0];
        f1IC = dot(e1, -Hinv*( (eye(4) - inv(Hinvtilde)*J*Hinv)*C + inv(Hinvtilde)*Jp'*qp ));
        g1IC = dot(e1, (1 - J'*inv(Hinvtilde)*J*Hinv)*Hinv*B);
        f1NC = dot(e1, -Hinv*C);
        g1NC = dot(e1, Hinv*B);
        
        Htilde = 1/2*zp^2 + 9.81*sind(beta)*(z-zmax); % H - Href
        
        sigmaRef = sqrt(-r^2 + (ox-x)^2 + (oz-z)^2);
        psiRef = atan2( -r*(ox-x) + sigmaRef*(oz-z) , -sigmaRef*(ox-x) - r*(oz-z) );
        
        JRigidinv = [((ox+(-1).*x).^2+(oz+(-1).*z).^2).^(-1).*(ox.^2+(-1).*r.^2+(-2).*ox.*x+x.^2+(oz+(-1).*z).^2).^(-1/2).*(ox.*r+(-1).*r.*x+(ox.^2+(-1).*r.^2+(-2).*ox.*x+x.^2+(oz+(-1).*z).^2).^(1/2).*((-1).*oz+z)),((ox+(-1).*x).^2+(oz+(-1).*z).^2).^(-1).*(ox.^2+(-1).*r.^2+(-2).*ox.*x+x.^2+(oz+(-1).*z).^2).^(-1/2).*(oz.*r+(ox+(-1).*x).*(ox.^2+(-1).*r.^2+(-2).*ox.*x+x.^2+(oz+(-1).*z).^2).^(1/2)+(-1).*r.*z);((-1).*ox+x).*(ox.^2+(-1).*r.^2+(-2).*ox.*x+x.^2+(oz+(-1).*z).^2).^(-1/2),(ox.^2+(-1).*r.^2+(-2).*ox.*x+x.^2+(oz+(-1).*z).^2).^(-1/2).*((-1).*oz+z)];
        
        temp = JRigidinv*[xp; zp];
        psipRef = temp(1);
        sigmapRef = temp(2);
        
        rhoRef = sigmaRef*cos(psiRef);
        rhopRef = sigmapRef*cos(psiRef) - sigmaRef*psipRef*sin(psiRef);
        rhobar = -ox;
        
        kappa0 = 0.0620;
        kappa1 = -0.0501;
        kappa00 = 0.0152;
        kappa01 = 0.0162;
        
        
        kappa0 = 0.1;
        kappa1 = -0.02;
        kappa00 = 0.01;
        kappa01 = 0.02;

        % Hand-tuned gains
%         kappa0 = 0.15;
%         kappa1 = -0.01;
%         kappa00 = 0.04;
%         kappa01 = 0.02;
        
        psid = -(kappa0 + kappa1*Htilde)*psiRef - ...
            ( kappa00*(rhoRef - rhobar) + kappa01*rhopRef );
        
        v = -obj.kp*(q(1)-psid) - obj.kd*qp(1);
%         u = v;
        
        count = evalin('base', 'count');
        if m == 2
            if count > 1000
                u = -f1IC/g1IC + v/g1IC;
            else
                u = -f1NC/g1NC + v/g1NC;
% %                 u = - (obj.kp*10*(q(1)-psid) + obj.kd*sqrt(10)*qp(1) )/g1NC;
            end
            count = count+1;
            assignin('base', 'count', count);
        else
            u = -f1NC/g1NC + v/g1NC;
            assignin('base', 'count', 0);
        end
        
        
%         uold = evalin('base', 'uold');
%         if (t > 0.1 && abs(uold - u) > 100 )
%             u = uold;
%         end
%         assignin('base', 'uold', u);
        
        if rem(t,10) < 0.5
            fprintf(['t = ', num2str(t), '\n'])
        end
    end 
    
  end
  
  methods (Static)
    function run(time,showGraphs)
      if nargin <2
        showGraphs = true;
      if nargin <1
        time = 10;
      end
      end
      p = SoftPaddleHybridReal();
      porig = p;
      plantSim = SimulinkModel(p.getModel());
      % take the frames from the simulink model and use those for the simulation of plant and controller
      p = setOutputFrame(p, getOutputFrame(plantSim));
      p = setInputFrame(p, getInputFrame(plantSim));
      
      c = SoftPaddleControlReal_v2(p);
      
      output_select(1).system = 1;
      output_select(1).output = plantSim.getOutputFrame();
      output_select(2).system = 2;
      output_select(2).output = c.getOutputFrame();
      
      sys = mimoFeedback(plantSim,c,[],[],[],output_select);
      
      x0 = p.getInitialState();
      
      tic
      [ytraj,xtraj] = simulate(sys,[0 time],x0);
      toc
      
      %extract utraj out of ytraj
      utraj = ytraj(10); % take last element of ytraj, because that contains u
      
      utraj = utraj.setOutputFrame(getInputFrame(porig));
      xtraj = xtraj.setOutputFrame(getOutputFrame(porig));
      
      v = porig.constructVisualizer();
      v.drawWrapper(0,x0);
      %TO record a trajectory, uncomment this next line:
      %v.playbackAVI(xtraj,'juggling_stabilized');
      v.playback(xtraj,struct('slider',true));
      
      tt=getBreaks(xtraj);
      
      %save('trajectorytoStabilizeShort.mat','utraj','xtraj','tt');
      
      %To print out some statistics
      if (showGraphs)
        % energy / cable length plotting
        % note, set alpha=0 in Manipulator/computeConstraintForce to reveal
        % some artifacts, especially at the release guard when
        % the pulley effectively hits a hard-stop.  this is due to the fact
        % that phidot>0 during the in_contact phase.  it's hard to tell if
        % this is numerical artifact or a bad gradient in CableLength.
        
        E=tt;
        cl=tt;
        nq=getNumPositions(porig.no_contact);
        dcl=zeros(length(tt),1,nq);
        ddcl=zeros(length(tt),1,nq*nq);
        for i=1:length(tt)
          x = xtraj.eval(tt(i));
          
          %           if i 50>= length(tt)/20 keyboard; end
          [T,U] = energy(porig,x);
          E(i)= T+U;
          if (x(1)==1) %flight mode
            [cl(i),dcl(i,1,:),ddcl(i,1,:)]=porig.no_contact.position_constraints{1}.fcn.eval(x((1:nq)+1));
          else
            [cl(i),dcl(i,1,:),ddcl(i,1,:)]=porig.in_contact.position_constraints{1}.fcn.eval(x((1:nq)+1));
          end
        end
        y = ytraj.eval(tt);
        figure(1); clf;
        subplot(3,1,1); plot(tt,E, 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\mathcal{H}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,2); plot(tt,cl, 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\phi(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,3); plot(tt,y(10,:), 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$u$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        figure(3); clf;
        subplot(3,1,1); plot(tt,cl, 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\phi(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,2); plot(tt,dcl(:,1,4), 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\dot{\phi}(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,3); plot(tt,ddcl(:,1,16), 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\ddot{\phi}(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        t = linspace(xtraj.tspan(1), xtraj.tspan(end), 1001);
        x = eval(xtraj, t);
        figure(2), clf
        subplot(2,2,1)
        plot(t,x(3,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\theta$ [rad]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,2)
        plot(t,x(4,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$x$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,3)
        plot(t, x(5,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$z$ [m]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,4)
        plot(t, x(2,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\psi$ [rad]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        figure(6)
        plot(t, x(6,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\dot{\psi}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        if (0)
            figure(25)
            v.playbackAVI(xtraj, 'soft_juggler_nice_stabilization')
        end
      end
    end
    
  end
end