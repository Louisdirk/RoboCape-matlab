
classdef CdcControllerAdapter < Controller

 
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
        originalController;
        car;
        dt;
        lookAhead;
        u1sat = inf;
    end
    
    
    methods
        
        
        function obj = CdcControllerAdapter(law,car,lookAhead,dt,u1sat)
            
            obj.originalController = law;
            obj.car       = car;
            obj.lookAhead = lookAhead;
            obj.dt        = dt;
            obj.u1sat     = u1sat;
            
        end
        
        function  u = computeInput(obj,t,x,varargin)
            aBetaDot = obj.originalController.computeInput(t,x);
            u        = obj.aBetaDot2vDelta(x,aBetaDot);
            u(1)     = max(min(u(1),obj.u1sat),-obj.u1sat);
            
        end
        
        function vDelta = aBetaDot2vDelta(obj,x,aBetaDot)
            
            a       = aBetaDot(1);
            betaDot = aBetaDot(2);
            
            v    = x(3);
            beta = x(5);
            l    = obj.car.l;
            lr   = obj.car.lr;
            d    = obj.lookAhead;
            
            nextV     = v     + d*a;
            nextBeta  = beta  + d*betaDot;
            nextDelta = atan((l/lr)*tan(nextBeta));
            
            vDelta = [nextV;nextDelta];
        end
        
        function aBetaDot = vDelta2aBetaDot(obj,x,vDelta)
            
            aBetaDot = [obj.vDelta2a(x,vDelta);
                        obj.vDelta2betaDot(x,vDelta)];
            
        end
        
        function a = vDelta2a(obj,x,vDelta)
            vRef    = vDelta(1);
            v       = x(3);
            dt      = obj.dt;
            a       = (vRef-v)/dt;
            
        end
        
        function betaDot = vDelta2betaDot(obj,x,vDelta)
            deltaRef = vDelta(2);
            beta     = x(5);
            l        = obj.car.l;
            lr       = obj.car.lr;
            dt       = obj.dt;
            betaRef  = atan((lr/l)*tan(deltaRef));
            betaDot  = (betaRef-beta)/dt;
        end
        
        
        
    end
end