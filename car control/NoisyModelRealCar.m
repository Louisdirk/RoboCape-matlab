
classdef NoisyModelRealCar < CtSystem & InitDeinitObject & ParameterizedCtSystem
% 'Parameters' [kv,kd,gv,gdelta]
%     dx = [
%         cos(x(4)+beta(x))*x(3);  %xDot
%         sin(x(4)+beta(x))*x(3);  %yDot
%         -kv*(x(3)-gv*u(1))    ;  %vDot     (first order model approximation)
%         (x(3)/lr)*sin(beta(x));  %phiDot
%         -kd*(x(5)-gdelta*u(2));  %deltaDot (first order model approximation)
%         ];
    properties 
        l = 0.096;
        lr = 0.056;
    end
    
    methods
        
        function obj = NoisyModelRealCar(Q,R,varargin)
            
            obj = obj@CtSystem(... 
            'nx',5,'nu',2,'ny',3,varargin{:});
            obj.parameters = [(1/0.7);(1/0.1);1;1];
            obj.f = @(t,x,u)obj.parametricF(t,x,u,obj.parameters)+chol(Q)'*randn(5,1);
            
            
            if obj.ny == 2
                obj.h =@(t,x)x([1,2])+chol(R)'*randn(2,1);
            else
                obj.h = @(t,x)x([1,2,4])+chol(R)'*randn(3,1);
            end
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                            
                        case 'Parameters'
                            
                            obj.parameters = varargin{parameterPointer+1};
                            
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
        
        function dx = parametricF(obj,t,x,u,p)
            
            kv     = p(1);
            kd     = p(2);
            gv     = p(3);
            gdelta = p(4);
            
            lr = obj.lr;
            l  = obj.l;
            beta = @(x)atan((lr/l)*tan(x(5)));

            dx = [
                cos(x(4)+beta(x))*x(3);  %xDot
                sin(x(4)+beta(x))*x(3);  %yDot
                -kv*(x(3)-gv*u(1))       ;  %vDot     (first order model approximation)
                (x(3)/lr)*sin(beta(x));  %phiDot
                -kd*(x(5)-gdelta*u(2))       ;  %deltaDot (first order model approximation)
            ];
        end
        
        function hP = plot(obj,p,theta)
            clr = 1;
        R = @(theta)[cos(theta),-sin(theta);
                     sin(theta),cos(theta)];
        h = obj.l;
        w = h/(2);
        k = 0.5;
           XY=[-w/2 , -h/2; 
               w/2 , -h/2 ; 
               k*w/2 , h/2 ; 
              -k*w/2 , h/2]*R(theta-pi/2)';

       hP=patch(XY(:,1)+p(1),XY(:,2)+p(2),clr);
  
        end
      
    end
    
end