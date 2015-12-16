classdef CircularTrajectory < handle
    
    properties
        center = [0,0];
        radius = 0;
        start_angle = 0;
        alpha = 0;
        omega = 0;
        
        end_of_traj = 0;
    end % properties
    
    methods
        
        function obj = CircularTrajectory(center,radius,phi,alpha,vf)
            
            obj.center = center;
            obj.radius = radius;
            obj.start_angle = phi - pi/2;
            obj.alpha = alpha;
            obj.omega = vf/radius;
            
        end % CircularTrajectory
        
        function [pos,vel,acc] = getTrajFromTime(obj,t)
            
            c = obj.center;
            r = obj.radius;
            s = obj.start_angle;
            a = obj.alpha;
            w = obj.omega;
            
            pos = [r*cos(w*t + s),      r*sin(w*t + s)     ] + c;
            vel = [-r*w*sin(w*t + s),   r*w*cos(w*t + s)   ];
            acc = [-r*w^2*cos(w*t + s), -r*w^2*sin(w*t + s)];
            
            if (w*t) >= (pi - 2*a)
                obj.end_of_traj = 1;
            else
                obj.end_of_traj = 0;
            end
            
        end % getTrajFromTime
        
        function ret = isEnd(obj)
            ret = obj.end_of_traj;
        end % isEnd
        
    end % methods
    
end % class