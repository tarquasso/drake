function [xdot,dxdot,ps,qs,qds,us,ts] = dynamicsSym(obj,t,x,u)
    % Provides the DrakeSystem interface to the manipulatorDynamics.
        nq = obj.num_positions;
        p = obj.getParamFrame.getPoly;
        np = length(p);
        q = x(1:nq);
        qd = x(nq+1:end);
        c = cos(q);  s = sin(q);
        if ~isempty(obj.position_constraints) || ~isempty(obj.velocity_constraints)
          % by naming this 'MATLAB:TooManyOutputs', geval will catch the
          % error and use TaylorVarInstead
          error('MATLAB:TooManyOutputs','User gradients for constrained dynamics not implemented yet.');
        end

        ps = sym('p',[np,1]);
        qs = sym('q',[nq,1]);
        qds= sym('qd',[nq,1]);
        ss = sin(qs);
        cs = cos(qs);
        us = sym('u',1);
        ts = sym('t',1);

        assume(ps,'real');
        assumeAlso(ps>=0);
        assume(qs,'real');
        assume(qds,'real');
        assume(us,'real');
        assume(ts,'real');
        assumeAlso(ts>=0);
        
        [H,C,B] = obj.manipulatorDynamics(q,qd);
        mssvars = getmsspoly([p;q;qd;s;c;u;t]);
        symvars = [ps;qs;qds;ss;cs;us;ts];
        
        if isa(H,'TrigPoly') 
            Hsym = msspoly2sym(mssvars,symvars,getmsspoly(H));
        elseif isa(H,'msspoly')
            Hsym = msspoly2sym(mssvars,symvars,H);
        else
            Hsym = H;
        end

        if isa(C,'TrigPoly') 
            Csym = msspoly2sym(mssvars,symvars,getmsspoly(C));
        elseif isa(C,'msspoly')
            Csym = msspoly2sym(mssvars,symvars,C);
        else
            Csym = C;
        end

        if isa(B,'TrigPoly') 
            Bsym = msspoly2sym(mssvars,symvars,getmsspoly(B));
        elseif isa(B,'msspoly')
            Bsym = msspoly2sym(mssvars,symvars,B);
        else
            Bsym = B;
        end

        xdot = qds;
        dxdot = Hsym\(Bsym*us-Csym);
    end
% 
% 
% function [xdot,dxdot,ps,qs,qds,us,ts] = dynamicsSym(H,C,B)
% %DYNAMICSSYM Summary of this function goes here
% %   Detailed explanation goes here
%   ps = sym('p',[np,1]);
%   qs = sym('q',[nq,1]);
%   qds= sym('qd',[nq,1]);
%   ss = sin(qs);
%   cs = cos(qs);
%   us = sym('u',1);
%   ts = sym('t',1);
% 
%   assume(ps,'real');
%   assume(ps>=0);
%   assume(qs,'real');
%   assume(qds,'real');
%   assume(us,'real');
%   assume(ts,'real');
%   assume(ts>=0);
%   
%   symvars = [ps;qs;qds;ss;cs;us;ts];
%   
% end
% function  = dynamicsSym()
%     % Provides the DrakeSystem interface to the manipulatorDynamics.
%         nq = obj.num_positions;
%         p = obj.getParamFrame.getPoly;
%         np = length(p);
%         q = x(1:nq);
%         qd = x(nq+1:end);
%         c = cos(q);  s = sin(q);
%         if ~isempty(obj.position_constraints) || ~isempty(obj.velocity_constraints)
%           % by naming this 'MATLAB:TooManyOutputs', geval will catch the
%           % error and use TaylorVarInstead
%           error('MATLAB:TooManyOutputs','User gradients for constrained dynamics not implemented yet.');
%         end
% 
% 
%         
%         mssvars = getmsspoly([p;q;qd;s;c;u;t]);
%         
%         if isa(H,'TrigPoly') 
%             Hsym = msspoly2sym(mssvars,symvars,getmsspoly(H));
%         elseif isa(H,'msspoly')
%             Hsym = msspoly2sym(mssvars,symvars,H);
%         else
%             Hsym = H;
%         end
% 
%         if isa(C,'TrigPoly') 
%             Csym = msspoly2sym(mssvars,symvars,getmsspoly(C));
%         elseif isa(C,'msspoly')
%             Csym = msspoly2sym(mssvars,symvars,C);
%         else
%             Csym = C;
%         end
% 
%         if isa(B,'TrigPoly') 
%             Bsym = msspoly2sym(mssvars,symvars,getmsspoly(B));
%         elseif isa(B,'msspoly')
%             Bsym = msspoly2sym(mssvars,symvars,B);
%         else
%             Bsym = B;
%         end
% 
%         xdot = qds;
%         dxdot = Hsym\(Bsym*us-Csym);
%     end
