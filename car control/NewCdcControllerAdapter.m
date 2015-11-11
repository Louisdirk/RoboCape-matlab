
classdef NewCdcControllerAdapter < ControllerAdapter
    
    % This file is part of VirtualArena.
    %
    % Copyright (c) 2014, Andrea Alessandretti
    % All rights reserved.
    %
    % e-mail: andrea.alessandretti [at] {epfl.ch, ist.utl.pt}
    %
    % Redistribution and use in source and binary forms, with or without
    % modification, are permitted provided that the following conditions are met:
    %
    % 1. Redistributions of source code must retain the above copyright notice, this
    %    list of conditions and the following disclaimer.
    % 2. Redistributions in binary form must reproduce the above copyright notice,
    %    this list of conditions and the following disclaimer in the documentation
    %    and/or other materials provided with the distribution.
    %
    % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    % ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    % WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    % DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
    % ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    % (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    % LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    % ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    % (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    % SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    %
    % The views and conclusions contained in the software and documentation are those
    % of the authors and should not be interpreted as representing official policies,
    % either expressed or implied, of the FreeBSD Project.
    properties
        car;
        u1sat;
        u2sat;
        
        Ke;
    end
    
    
    methods
        
        
        function obj = NewCdcControllerAdapter(law,car,u1sat,u2sat)
            
            obj = obj@ControllerAdapter(law);
            obj.car                = car;
            obj.u1sat = u1sat;
            obj.u2sat = u2sat;
            obj.Ke = blkdiag(law.K,law.kxi);
            
        end
        
        function originalX = x2originalX(obj,t,x)
            %x =[x;y;v;phi;delta] > originalX = [x;y;v;phi;beta]
            
            lr = obj.car.lr;
            l = obj.car.l;
            
            delta  = x(5);
            beta   = atan( (lr/l)*tan(delta) );
            
            originalX    = x;
            originalX(5) = beta;
            
        end
        
        function u = originalU2newU(obj,t,x,originalU)
            % originalU = [a;betaDot] > u =[vRef;deltaRef]
            %
            % dotV    = a = -kv*(x(3)-u(1))
            % betaDot = (dBeta/dDelta)*(-kd*(x(5)-u(2)))
            
            a       = originalU(1);
            betaDot = originalU(2);
            delta   = x(5);
            v       = x(3);
            
            kv = obj.car.parameters(1);
            kd = obj.car.parameters(2);
            l  = obj.car.l;
            lr = obj.car.lr;
            
            dBetadDelta = @(delta) (lr/l)*sec(delta)^2/(1+((lr/l)*tan(delta))^2);
            
            u = [(1/kv                     )*(a       + kv                     *v    );
                (1/(kd*dBetadDelta(delta)))*(betaDot + (kd*dBetadDelta(delta))*delta)];
            
            if ~isempty(obj.u1sat)
                u(1) = max(min(u(1),obj.u1sat),0);
            end
            if ~isempty(obj.u2sat)
                u(2) = max(min(u(2),obj.u2sat),-obj.u2sat);
            end
        end
        
        function e = computeError(obj,t,x)
            e = obj.originalController.getLyapunovVariable(t,x);
        end
        
    end
end