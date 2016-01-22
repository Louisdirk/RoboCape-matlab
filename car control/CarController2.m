classdef CarController2 < Controller
    %InlineController inline Controller
    %
    %   InlineController peoperties:
    %
    %   law     - @(t,x) function handle of the control law
    %
    %   InlineController methods:
    %
    %   InlineController - constructor, c = InlineController(@law)
    %
    %   See also Controller
    
    
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
    
    properties % (Access = protected)
        pd;
        pdDot;
        pdDDot;
        epsilon;
        lr;
        l;
        K=eye(2);
        kxi = 1;
        
        u1sat = inf;
        u2sat = inf;
        kv;
        kd;
        
        lastE;
    end
    
    
    methods
        
        
        function obj = CarController2(varargin)
            
            obj = obj@Controller();
            
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        
                        case 'Epsilon'
                            
                            obj.epsilon = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'pd'
                            
                            obj.pd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'pdDot'
                            
                            obj.pdDot = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'pdDDot'
                            
                            obj.pdDDot = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Ke'
                            
                            obj.K = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Kxi'
                            
                            obj.kxi = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                        case 'lr'
                            
                            obj.lr = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'l'
                            
                            obj.l = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'u1sat'
                            
                            obj.u1sat = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'u2sat'
                            
                            obj.u2sat = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'kv'
                            
                            obj.kv = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'kd'
                            
                            obj.kd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
                
                
            end
            
        end
        
        % x= [p;v;phi,beta]
        function u = computeInput(obj,t,x)
            
            epsilon = obj.epsilon;
            ke      = obj.K;
            kz      = obj.kxi;
            lr      = obj.lr;
            l       = obj.l;
            kv      = obj.kv;
            kd      = obj.kd;
            pd      = obj.pd(t);
            pdDot   = obj.pdDot(t);
            pdDDot  = obj.pdDDot(t);
            
            hatE = getLyapunovVariable(obj,t,x);
            e  = hatE(1:2);
            z = hatE(3);
            
            
            beta = @(x) atan((lr/l)*tan(x(5)));
            phiDot = @(x) (x(3)/l)*cos(beta(x))*tan(x(5));
            
            R = [cos(x(4)+beta(x)),-sin(x(4)+beta(x));
                sin(x(4)+beta(x)), cos(x(4)+beta(x))];
            
            invDelta = [
                        1   epsilon(2)/epsilon(1);
                        0   1/epsilon(1)
                        ];
            
            omega = invDelta(2,:)*(R'*pdDot - ke*e);
            
            betaDot = omega - phiDot(x);
            
            S = [0   , -omega;
                omega, 0     ];
            
            N = invDelta(1,:)*(-S*R'*pdDot+ R'*pdDDot - ke*((-S-ke)*e + [z;0]));
            vfDot = - e(1) + N - kz*z;
            obj.lastE =  [e;z];
            
            dBetadDelta = (lr/l)*sec(x(5))^2/(1+((lr/l)*tan(x(5)))^2);
            deltaDot = betaDot/dBetadDelta;
            
            u = [vfDot/kv + x(3);
                deltaDot/kd + x(5)];
            
            if ~isempty(obj.u1sat)
%                 u(1) = max(min(u(1),obj.u1sat),0);
                    u(1) = max(min(u(1),obj.u1sat),-obj.u1sat);
            end
            if ~isempty(obj.u2sat)
                u(2) = max(min(u(2),obj.u2sat),-obj.u2sat);
            end
            
        end
        
        
        function hatE= getLyapunovVariable(obj,t,x)
            
            K       = obj.K;
            pd      = obj.pd(t);
            pdDot   = obj.pdDot(t);
            epsilon = obj.epsilon;
            lr      = obj.lr;
            l       = obj.l;
            
            beta = @(x) atan((lr/l)*tan(x(5)));
            
            invDelta = [
                        1   epsilon(2)/epsilon(1);
                        0   1/epsilon(1)
                        ];
            
            R = [cos(x(4)+beta(x)),-sin(x(4)+beta(x));
                sin(x(4)+beta(x)), cos(x(4)+beta(x))];
            
            e = R'*(x(1:2)-pd)+epsilon;
            
            z = x(3)-invDelta(1,:)*(-K*e+R'*pdDot);
            
            hatE=[e;z];
        end
        
        function n= getLyapunovValue(obj,t,x)
            
            hatE = obj.getLyapunovVariable(t,x);
            n    = 0.5*hatE'*hatE;
        end
        
    end
    
end