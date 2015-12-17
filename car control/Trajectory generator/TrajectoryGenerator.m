classdef TrajectoryGenerator < handle
    
    properties
        radius = 0;
        aCircle = 0;                            % Maximum centripetal acceleration
        
        aMax = 0;                               % Maximum forward acceleration
        vSat = 0;                               % Maximal velocity
        
        checkpoints = [];
        checkpointsSpeed = [];
        nCheckpoints = 0;
        
        traj = {};
        
        tP = 0;
        currentPart = 1;
    end % properties
    
    
    methods
        
        function obj = TrajectoryGenerator(checkpoints,checkpointsSpeed,radius,aCircle,aMax,vSat)
            
            obj.checkpoints = checkpoints;
            nCheckpoints = size(checkpoints,1);
            obj.nCheckpoints = nCheckpoints;
            vCircle = sqrt(aCircle*radius);         % Max speed in curve
            
            checkpointsSpeed = min(checkpointsSpeed, vCircle*ones(nCheckpoints,1));
            
            if size(checkpoints,1) ~= length(checkpointsSpeed)
                error('size(checkpointsSpeed,1) ~= length(checkpoints)');
            end
            
            obj.checkpointsSpeed = checkpointsSpeed;
            obj.radius = radius;
            obj.aCircle = aCircle;
            obj.aMax = aMax;
            obj.vSat = vSat;
            
            [l2,dir,center,phi,alpha,turnDir] = computeGeometry(obj);
            obj.traj = createCompleteTraj(obj,l2,dir,center,phi,alpha,turnDir);
            
        end % TrajectoryGenerator
        
        function [l2,dir,center,phi,alpha,turnDir] = computeGeometry(obj)
            
            radius = obj.radius;
            aCircle = obj.aCircle;
            aMax = obj.aMax;
            vSat = obj.vSat;
            p = obj.checkpoints;
            pv = obj.checkpointsSpeed;
            nCheckpoints = obj.nCheckpoints;
            
            dir(:, nCheckpoints) = [0;0];
            for n = 1:(nCheckpoints-1)
                dir(:,n) = points2vectU(p(n,:),p(n+1,:)); % Computes unitary direction vectors.
                l(n) = norm(p(n+1)-p(n));
            end
            
            alpha = zeros(nCheckpoints,1);
            l2 = zeros(nCheckpoints,1);
            alpha(1) = 0;
            l2(1) = 0;
            l2(nCheckpoints) = 0;
            psi(1) = vectU2psi([1;0],dir(:,1));
            phi(1) = psi(1);
            
            for n = 2:(nCheckpoints-1)
                psi(n) = vectU2psi(dir(:,n-1),dir(:,n));
                alpha(n) = psi2alpha(abs(psi(n)));
                phi(n) = wrapTo2Pi(phi(n-1)+psi(n));
                
                l2(n) = abs(radius/tan(alpha(n)));
            end
            
            for n = 1:(nCheckpoints-1)
                l1(n) = l(n) - l2(n) - l2(n+1);
            end
            
            % Compute circle parameters
            for n = 2:(nCheckpoints-1)
                turnDir(n) = sign(det([dir(:,n-1),dir(:,n)]));
                center(n,:) = p(n,:)' - l2(n)*dir(:,n-1) + turnDir(n)*radius*[-dir(2,n-1);dir(1,n-1)];
            end
            
            lTot = sum(l1) + sum(radius*2*alpha);   % Total path length
            
        end % computeGeometry
        
        function traj = createCompleteTraj(obj,l2,dir,center,phi,alpha,turnDir)
            
            radius = obj.radius;
            aCircle = obj.aCircle;
            aMax = obj.aMax;
            vSat = obj.vSat;
            p = obj.checkpoints;
            pv = obj.checkpointsSpeed;
            nCheckpoints = obj.nCheckpoints;
            
            nSegments = nCheckpoints-1;
            nCircles = nCheckpoints-1;
            index = 1;
            indexC = 2;
            
            for n = 1:2:2*(nSegments)
                p0 = p(index,:) + l2(index)*dir(:,index)';
                pf = p(index+1,:) - l2(index+1)*dir(:,index)';
                traj{n} = StraightTrajectory(p0,pf,pv(index),pv(index+1),aMax,vSat);
                traj{n+1} = CircularTrajectory(center(indexC,:),radius,phi(indexC-1),alpha(indexC),traj{n}.vf,turnDir(indexC));
                
                if indexC < nCircles
                    indexC = indexC + 1;
                else
                    indexC = 2;
                end
                index = index + 1;
            end
            
        end % createCompleteTraj
        
        function [pos,vel,acc] = getTrajFromTime(obj,t)
            
            nCheckpoints = obj.nCheckpoints;
            
            nSegments = nCheckpoints-1;
            nCircles = nCheckpoints-1;
            currentPart = obj.currentPart;
            
            % Load
            tP = obj.tP;
            currentPart = obj.currentPart;
            
            % Get trajectory values
            if (obj.traj{currentPart}.isEnd() == 1) && (currentPart < (nSegments + nCircles-1))
                currentPart = currentPart + 1;
                tP = t;
            end
            tLoc = t - tP;
            [pos, vel, acc] = obj.traj{currentPart}.getTrajFromTime(tLoc);
            
            % Store
            obj.tP = tP;
            obj.currentPart = currentPart;
            
        end % getTrajFromTime
        
    end % methods
    
end % class