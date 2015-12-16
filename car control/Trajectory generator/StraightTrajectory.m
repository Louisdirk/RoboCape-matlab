classdef StraightTrajectory < handle
    
    properties
        
        p0 = [0,0];
        pf = [0,0];
        v0 = 0;
        vf = 0;
        aMax = 0;;
        vSat = 0;
        dirVectU = [1;0];
        l = 0;
        
        T1 = 0;
        T2 = 0;
        T3 = 0;
        
        vt1 = 0;
        vt3 = 0;
        xt1 = 0;
        xt2 = 0;
        xt3 = 0;
        
        end_of_traj = 0;
        
    end % properties
    
    
    methods
        
        function obj = StraightTrajectory(p0,pf,v0,vf,aMax,vSat)
            
            obj.p0 = p0;
            obj.pf = pf;
            obj.v0 = v0;
            obj.vf = vf;
            obj.aMax = aMax;
            obj.vSat = vSat;
            obj.dirVectU = points2vectU(p0,pf);
            obj.l = norm(pf-p0);
            
            x0 = 0;
            
            [T1,T2,T3,vf] = computeSegmentTrajectory(x0,obj.l,v0,vf,aMax,vSat);
            
            vt1 = aMax*T1 + v0;
            vt3 = aMax*(T1-T3) + v0;
            xt1 = 1/2*aMax*T1^2 + v0*T1 + x0;
            xt2 = aMax*T1*T2 + v0*T2 + xt1;
            xt3 = -1/2*aMax*T3^2 + vt1*T3 + xt2;
            
            obj.vt1 = vt1;
            obj.vt3 = vt3;
            obj.xt1 = xt1;
            obj.xt2 = xt2;
            obj.xt3 = xt3;
            
            obj.T1 = T1;
            obj.T2 = T2;
            obj.T3 = T3;
            obj.vf = vf;
            
            obj.end_of_traj = 0;
            
        end % method StraighttTrajectory
        
        
        function [pos,vel,acc] = getTrajFromTime(obj,t)
            
            vt1 = obj.vt1;
            xt1 = obj.xt1;
            xt2 = obj.xt2;
            
            T1 = obj.T1;
            T2 = obj.T2;
            T3 = obj.T3;
            
            x0 = 0;
            xf = obj.l;
            v0 = obj.v0;
            vf = obj.vf; 
            
            aMax = obj.aMax;
            
            if t <= T1
                tLoc = t;
                a = aMax;           %#ok<*PROP>
                v = aMax*tLoc + v0; %#ok<*CPROP>
                x = 1/2*aMax*tLoc^2 + v0*tLoc + x0;
                end_of_traj = 0;
            elseif t <= T1+T2
                tLoc = t-T1;
                a = 0;
                v = vt1;
                x = vt1*tLoc + xt1;
                end_of_traj = 0;
            elseif t < T1+T2+T3
                tLoc = t-T1-T2;
                a = -aMax;
                v = -aMax*tLoc + vt1;
                x = -1/2*aMax*tLoc^2 + vt1*tLoc + xt2;
                end_of_traj = 0;
            else
                a = -aMax;
                v = vf;
                x = xf;
                end_of_traj = 1;
            end
            obj.end_of_traj = end_of_traj;
            
            % Transpose into x-y frame
            pos = (x*obj.dirVectU)' + obj.p0;
            vel = (v*obj.dirVectU)';
            acc = (a*obj.dirVectU)';            
            
        end % method getTrajFromTime
        
        
        function ret = isEnd(obj)
            
            ret = obj.end_of_traj;
            
        end
    end
    
end