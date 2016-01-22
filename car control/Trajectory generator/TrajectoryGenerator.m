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
        tPmem = [];
        
        lTot = 0;
        
        currentPart = 1;
        
        currentTime = -1;
        currentPos = [0;0];
        currentVel = [0;0];
        currentAcc = [0;0];
        
        trajProj = [0, 0, 0];
        xStart = 0;
        
    end % properties
    
    
    methods
        
        function obj = TrajectoryGenerator(checkpoints,checkpointsSpeed,radius,aCircle,aMax,vSat)
            
            obj.checkpoints = checkpoints;
            nCheckpoints = size(checkpoints,1);
            obj.nCheckpoints = nCheckpoints;
            vCircle = min(vSat,sqrt(aCircle*radius));         % Max speed in curve
            
            if size(checkpoints,1) ~= length(checkpointsSpeed)
                error('size(checkpointsSpeed,1) ~= length(checkpoints)');
            end
            
            checkpointsSpeed = min(checkpointsSpeed, vCircle*ones(nCheckpoints,1));
            
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
%             psi(1) = vectU2psi([1;0],dir(:,1));
            psi(1) = atan2(dir(2,1),dir(1,1));
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
            
            obj.lTot = sum(abs(l1)) + sum(radius*2*alpha);   % Total path length
            
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
            
            % Load
            tP = obj.tP;
            currentPart = obj.currentPart;
            xStart = obj.xStart;
            
            if t == 0
                currentPart = 1;
                tP = 0;
                for n = 1:(nSegments + nCircles-1)
                    obj.traj{n}.restart();
                end
            end
            
            % Get trajectory values
            if (obj.traj{currentPart}.isEnd() == 1) && (currentPart < (nSegments + nCircles-1))
                currentPart = currentPart + 1;
                tP = t;
                xStart = obj.trajProj(1);
            end
            tLoc = t - tP;
            [pos, vel, acc] = obj.traj{currentPart}.getTrajFromTime(tLoc);
            
            % Store
            obj.tP = tP;
            obj.currentPart = currentPart;
            obj.tPmem = [obj.tPmem t];
            obj.trajProj = obj.traj{currentPart}.getTrajProj()+[xStart, 0, 0];
            obj.xStart = xStart;
            
            
        end % getTrajFromTime
        
        function ret = getTrajProj(obj)
            ret = obj.trajProj;
        end
        
        function currentPos = getPosFromTime(obj,t)
            
            if t == obj.currentTime
                currentPos = obj.currentPos;
            else
                obj.currentTime = t;
                [currentPos, currentVel, currentAcc] = getTrajFromTime(obj,t);
                obj.currentPos = currentPos';
                obj.currentVel = currentVel';
                obj.currentAcc = currentAcc';
                currentPos = currentPos';
            end
        end
        
        function currentVel = getVelFromTime(obj,t)
            
            if t == obj.currentTime
                currentVel = obj.currentVel;
            else
                obj.currentTime = t;
                [currentPos, currentVel, currentAcc] = getTrajFromTime(obj,t);
                obj.currentPos = currentPos;
                obj.currentVel = currentVel;
                obj.currentAcc = currentAcc;
                currentVel = currentVel';
            end
            
        end
        
        function currentAcc = getAccFromTime(obj,t)
            
            if t == obj.currentTime
                currentAcc = obj.currentAcc;
            else
                obj.currentTime = t;
                [currentPos, currentVel, currentAcc] = getTrajFromTime(obj,t);
                obj.currentPos = currentPos;
                obj.currentVel = currentVel;
                obj.currentAcc = currentAcc;
                currentAcc = currentAcc';
            end
            
        end
        
        
    end % methods
    
end % class