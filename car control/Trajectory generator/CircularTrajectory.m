classdef CircularTrajectory < handle
    
    properties
        center = [0,0];
        radius = 0;
        start_angle = 0;
        alpha = 0;
        omega = 0;
        vf = 0;
        
        end_of_traj = 0;
        
        trajProj = [0, 0, 0];
    end % properties
    
    methods
        
        function obj = CircularTrajectory(center,radius,phi,alpha,vf,rotation)
            
            obj.center = center;
            obj.radius = radius;
            
            if rotation == 1
                obj.start_angle = phi - pi/2;
            else
                obj.start_angle = phi + pi/2;
            end
                
            obj.alpha = alpha;
            obj.omega = vf/radius*rotation;
            obj.vf = vf;
            
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
            
            obj.trajProj = [obj.vf*t, obj.vf, 0];
            
            if abs(w*t) >= (pi - 2*a)
                obj.end_of_traj = 1;
            else
                obj.end_of_traj = 0;
            end
            
        end % getTrajFromTime
        
        function ret = getTrajProj(obj)
            ret = obj.trajProj;
        end
        
        function ret = isEnd(obj)
            ret = obj.end_of_traj;
        end % isEnd
        
        function restart(obj)
            obj.end_of_traj = 0;
        end
        
    end % methods
    
end % class