classdef SoftPaddleControlRealMl < DrakeSystem
    properties
        kp
        kd
        plant
        kappa0
        kappa1
        kappa00
        kappa01
    end
    
    methods
        function obj = SoftPaddleControlRealMl(plant)
            obj = obj@DrakeSystem(0,0,9,1,true,true);
            obj = setInputFrame(obj, getOutputFrame(plant));
            obj = setOutputFrame(obj, getInputFrame(plant));
            obj.kp = 5000;
            obj.kd = 2*sqrt(obj.kp);
            obj.plant = plant;
            
            % Control gains
            obj.kappa0 = 0.6;
            obj.kappa1 = 0.2;
            obj.kappa00 = 0.8;
            obj.kappa01 = 0.235;
        end
        
        function u = output(obj,t,~,xState)
            ox = -0.145;
            r = obj.plant.radius;
            
            m = xState(1);
            q = xState(2:5);
            qp = xState(6:9);
            
            psi = q(1); x = q(3); z = q(4);
            psip = qp(1); xp = qp(3); zp = qp(4);
            
            % Kinematics and Dynamics for the soft paddle
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [H,C,B] = manipulatorDynamics(obj.plant.no_contact, q, qp);
            [phi,J,ddphi] = obj.plant.in_contact.position_constraints{1}.eval(q);
            
            Jp = reshape(ddphi,length(q),[])*qp;
            Hinv = inv(H);
            Hinvtilde = J*Hinv*J';
            Delta = Hinvtilde - J(1)*J*Hinv*B;
            
            [T,U] = energy(obj.plant,xState);
            Eref = 0.414;
            Etilde = T+U-Eref;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            % Inverse position and velocity kinematics for a rigid paddle
            %--------------------------------------------------------------------------------
            xiRef=[((-1).*r.^2+(ox+(-1).*x).^2+z.^2).^(1/2),
                atan2(r.*((-1).*ox+x)+(-1).*z.*((-1).*r.^2+(ox+(-1).*x).^2+z.^2).^(1/2), r.*z+(-1).*(ox+(-1).*x).*((-1).*r.^2+(ox+(-1).*x).^2+z.^2).^(1/2))];
            
            sigmaRef = xiRef(1);
            psiRef = xiRef(2);
            %--------------------------------------------------------------------------------
            JRigidInv=[((-1).*ox+x).*((-1).*r.^2+(ox+(-1).*x).^2+z.^2).^(-1/2),z.*((-1).*r.^2+(ox+(-1).*x).^2+z.^2).^(-1/2);((ox+(-1).*x).^2+z.^2).^(-1).*((-1).*r.^2+(ox+(-1).*x).^2+z.^2).^(-1/2).*(ox.*r+(-1).*r.*x+z.*((-1).*r.^2+(ox+(-1).*x).^2+z.^2).^(1/2)),((ox+(-1).*x).^2+z.^2).^(-1).*(ox+(-1).*x+(-1).*r.*z.*((-1).*r.^2+(ox+(-1).*x).^2+z.^2).^(-1/2))];

            xipRef = JRigidInv*[xp; zp];
            sigmapRef = xipRef(1);
            psipRef = xipRef(2);
            %--------------------------------------------------------------------------------

            % Reference for the x-motion of the ball
            rhoBar = ox;
            rhoRef = sigmaRef*cos(psiRef);
            rhopRef = sigmapRef*cos(psiRef) - sigmaRef*psipRef*sin(psiRef);

            % Vertical energy of the ball and its reference energy
            Href = 4.58;
            Htilde = Href - 1/2*zp^2 - 9.81*z;
            
            psid = -(obj.kappa0 + obj.kappa1*Htilde)*psiRef + ...
                obj.kappa00*(rhoRef - rhoBar) + obj.kappa01*rhopRef; 
           
%             psid = -(obj.kappa0 + obj.kappa1*Htilde)*psiRef + ...
%                 obj.kappa00*(sigmaRef - ox) + obj.kappa01*sigmapRef; 
            
            % Control the paddle so that psi --> psid
            u = -obj.kp*(q(1)-psid) - obj.kd*qp(1) + C(1);
            if m == 2
                k2 = 0;
                u = Hinvtilde/(Delta)*( -obj.kp*(q(1)-psid) - obj.kd*qp(1) + C(1) ) + J(1)*(Jp'*qp-J*Hinv*C)/(Delta) - k2*Etilde*qp(1);
            end
            
            % Observe the progress of the simulation every once in a while
            if rem(t,2) < 0.1
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
            
            c = SoftPaddleControlRealMl(p);
            
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
                
                t = linspace(xtraj.tspan(1), xtraj.tspan(end), 1001);
                x = eval(xtraj, t);
                figure(2), clf
                subplot(2,2,1:2), hold on
                plot(t,x(4,:), 'LineWidth', 2)
                plot(t, x(8,:), 'LineStyle', '-.')
                axis('tight')
                xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
                ylabel('$x$, $\dot{x}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
                h = legend('$x$', '$\dot{x}$', 'Location','Best');
                set(h,'Interpreter', 'LaTeX', 'FontSize', 15)
                subplot(2,2,3:4), hold on
                plot(t, x(5,:), 'LineWidth', 2)
                plot( t, x(9,:), 'LineStyle', '-.')
                axis('tight')
                xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
                ylabel('$z$, $\dot{z}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
                h = legend('$z$', '$\dot{z}$', 'Location','Best');
                set(h,'Interpreter', 'LaTeX', 'FontSize', 15)
                
                figure(3), clf
                subplot(2,2,1:2), hold on
                plot(t, x(2,:),'LineWidth', 2)
                plot( t, x(6,:), 'LineStyle', '-.')
                axis('tight')
                xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
                ylabel('$\psi$, $\dot{\psi}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
                h = legend('$\psi$', '$\dot{\psi}$', 'Location','Best');
                set(h,'Interpreter', 'LaTeX', 'FontSize', 15)
                subplot(2,2,3:4), hold on
                plot(t, x(3,:), 'LineWidth', 2)
                plot(t, x(7,:), 'LineStyle', '-.')
                axis('tight')
                xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
                ylabel('$\theta$, $\dot{\theta}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
                h = legend('$\theta$', '$\dot{\theta}$', 'Location','Best');
                set(h,'Interpreter', 'LaTeX', 'FontSize', 15)
                
                
                figure(4); clf;
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
                
%                 figure(5)
%                 plot(t, x(6,:), 'LineWidth', 2)
%                 axis('tight')
%                 xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
%                 ylabel('$\dot{\psi}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
                
                figure(2)
                
                if (0)
                    figure(25)
                    v.playbackAVI(xtraj, 'asymp_stable_soft_juggling_real_mirror_law')
                end
            end
        end
        
    end
end