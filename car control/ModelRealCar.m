
classdef ModelRealCar < CtSystem & InitDeinitObject & ParameterizedCtSystem
    % 'Parameters' [kv,kd,gv,gdelta]
    %     dx = [
    %         cos(x(4)+beta(x))*x(3);  %xDot
    %         sin(x(4)+beta(x))*x(3);  %yDot
    %         -kv*(x(3)-gv*u(1))    ;  %vDot     (first order model approximation)
    %         (x(3)/l)*cos(beta(x)*tan(x(4)));  %phiDot
    %         -kd*(x(5)-gdelta*u(2));  %deltaDot (first order model approximation)
    %         ];
    properties
        l = 0.34;
        lr = 0.17;
    end
    
    methods
        
        function obj = ModelRealCar (varargin)
            
            obj = obj@CtSystem(...
                'nx',5,'nu',2,'ny',3,varargin{:});
            if strcmp(varargin{5}, 'Parameters')
                obj.parameters = varargin{6};
            else
                obj.parameters = [(1/0.7);(1/0.1);1;1];
            end
            obj.f = @(t,x,u) obj.parametricF(t,x,u,obj.parameters);
            
            if obj.ny == 2
                obj.h =@(t,x) x([1,2]);
            elseif obj.ny == 3
                obj.h = @(t,x) x([1,2,4]);
            elseif obj.ny == 6
                obj.h = @(t,x,u) obj.hh(t,x,u);
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
        function y = hh(obj, t, x, u)
            kv = obj.parameters(1);
            kf = obj.parameters(2);
            gv = obj.parameters(3);
            gf = obj.parameters(4);
            
            lr = obj.lr;
            l  = obj.l;
            
            beta = @(x)atan((lr/l)*tan(x(5)));
            
            % Output vector y = h(x,u)
            y = [
                x(1);
                x(2);
                -kf*(x(3)-gv*u(1));
                x(3)^2*cos(beta(x))*tan(x(5))/l;
                x(3)*cos(beta(x))*tan(x(5))/l;
                x(4)
            ];
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
                cos(x(4)+beta(x))*x(3);         % xDot
                sin(x(4)+beta(x))*x(3);         % yDot
                -kv*(x(3)-gv*u(1));             % vDot     (first order model approximation)
                 (x(3)/l)*cos(beta(x))*tan(x(5));% phiDot
                -kd*(x(5)-gdelta*u(2));         % deltaDot (first order model approximation)
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