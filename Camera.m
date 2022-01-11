% Copyright (c) 2019, Nicolò Bargellesi, Luca Facin & Lorenzo Marchini
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree.
%

classdef Camera < handle
   properties
      Task  % Patrolling(0)/Tracking(1)
      Theta % Field of View Angle
      FoV   % Field of View
      zmin  % Minimum Height
      zMax  % Maximum Height
      eFoV  % Detection Error (Percentage of FoV)
      V     % Vertex (in movement domain)
      X     % Position [x,y,z]
   end
   methods
       % CONSTRUCTOR
      function cam = Camera(env,theta,v,zbound,efov)
         if nargin > 0
            cam.Task = 0;
            cam.Theta = theta;
            cam.V = v;
            cam.FoV = env.Res;
            cam.eFoV = efov;
            cam.zMax = cam.FoV/tan(theta*pi/180);
            %cam.zmin = zbound;
            cam.zmin = cam.zMax/4;
            cam.X = [env.Vm(cam.V,1), env.Vm(cam.V,2), cam.zMax];
         end
      end
      
      % HEIGHT UPDATE
      function updateZ(cam,newZ)
        if (newZ >= cam.zmin && newZ <= cam.zMax)
            cam.X(3) = newZ;
            cam.FoV = newZ*tan(cam.Theta*pi/180);
        end
      end
      
      % SEBS
      function nextV = SEBS(cam,env,R,S,L,M)
        vA = find(env.Am(cam.V,:));     % find adjacent vertices
        Htot = 0;
        for k = vA
           Htot = Htot + sum(env.H(logical(env.A(env.Vmap(k),:))));
        end
        PmGS = [];
        for k = vA
            %G = max(env.Iv(logical(env.A(env.Vmap(k),:))));     % (shortest length)
            G = sum(env.Iv(logical(env.A(env.Vmap(k),:))));     % (minimal average idleness)
            if (G > M)
                G = M;
            end
            % Convert Heat in Prior
            Pm = sum(env.H(logical(env.A(env.Vmap(k),:))))/Htot;         
            PGm = L * exp(log(1/L)/M*G);
            view = env.A(env.Vmap(k,:));
            view(1,env.Vmap(k)) = 1;
            s = sum(S(logical(view)));
            PSm = (2^(R-s-1))/(2^R-1);
            PmGS = [PmGS, Pm * PGm * PSm];
        end
       nextV = vA(find(PmGS-max(PmGS)>=0,1));    % new vertex selected
      end
      
      
      % OPTIMAL ZOOM
      function [z,k] = optimalZoom(cam,gamma,sd)
        A = [-tan(cam.Theta*pi/180) sd; 0 0];
        b = [0; 0];
        % pursuit of aceptable starting point
        f = zeros(1,2); % assumes x0 is the initial point
        x0 = linprog(f,A,b,[],[],[cam.zmin 2],[cam.zMax 3]);
        if isempty(x0)
            z =  cam.zMax;
            k = 2;
            return
        end
        % Information Lost (due to distance)
        I = @(z) -(exp(z-cam.zmin)-1);
        % Uncertainty Estimation
        beta = 0.6161;
        u = @(k) (1-exp(-k/beta));
        % Cost Function
        f = @(z,k) -(I(z)+gamma*u(k));
        fun = @(x) f(x(1),x(2));
        % Optimal Problem
        options = optimoptions('fmincon','Algorithm','interior-point');
        options.Display = 'notify';
        x = fmincon(fun,x0,A,b,[],[],[cam.zmin 2],[cam.zMax 3],[],options);
        % Solve
        z = x(1);
        k = x(2);
      end
        
      
      % PLOT
      function plot(cam)
        if cam.Task == 0
            scatter(cam.X(1),cam.X(2),30,[0.9290, 0.6940, 0.1250],'filled')
        else
            scatter(cam.X(1),cam.X(2),30,[0, 0.4470, 0.7410],'filled')
        end
        rectangle('Position',[cam.X(1)-cam.FoV cam.X(2)-cam.FoV 2*cam.FoV 2*cam.FoV])
        axis equal
      end
        
   end
end
