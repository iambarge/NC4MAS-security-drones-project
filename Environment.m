% Copyright (c) 2019, Nicolò Bargellesi, Luca Facin & Lorenzo Marchini
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree.
%

classdef Environment < handle
   properties
      B         % Borders
      NFZ       % No Fly Zones
      Res       % Resolution of Grid
      V         % Vertices Coordinates
      Vm        % Movement Vertices Coordinates
      Vmap      % Map: Movement Domain -> Environment Domain
      E         % Edges
      Em        % Movement Edges
      A         % Adjacency Matrix of E (vision)
      Am        % Adjacency Matrix of Em (movement)
      H         % Heat Map of Vertices
      
      % For Binary Classification
      cycles    % Simple Cycles in (Vm,Em)
      
      % For SEBS
      tl        % Last vision time of vertices
      Iv        % Idleness of movement vertices
   end
   methods
      % CONSTRUCTOR
      function env = Environment(b,nfz)
        env.B = b;
        env.NFZ = nfz;
      end
      
      % CELLULARIZATION
      function cellularize(env,res)
        env.Res = res;
        % Vertices
        [env.V,env.Vm,env.Vmap] = envCell(env.B,env.NFZ,res);
        % Initialize Heat
        env.heatReset();
        % Edges
        [env.Em,env.Am,env.A] = envEdges(env.V,env.Vm,env.Vmap,res);
        env.E = remapEdges(env.Em,env.Vmap); % Remap movement -> environment domain
        % Initialize Idleness
        env.IvReset();
      end
      
      % RESET HEAT
      function heatReset(env)
        env.H = ones(1,length(env.V));
      end
      
      % ADD HEAT
      function addHeat(env,ksize,alpha,sigma,v)
        % Define Gaussian Kernel
        [R,C] = ndgrid(1:2*ksize-1, 1:2*ksize-1);
        exponent = ((R-ksize).^2 + (C-ksize).^2)./(2*sigma);
        kernel = alpha*(exp(-exponent));
        % Allign to Grid
        if (ksize == 1)
            Vk = v;
        elseif (ksize > 1)
            Av = env.A;
            for k=2:ksize-1
                Av = sign(Av+(Av*env.A));
            end
            Vk = [v,find(Av(v,:))];
        else
            disp('Kernel size must be a positive number!')
        end
        % Update Heat
        for vk = Vk
            d = [env.V(vk,1)-env.V(v,1), env.V(vk,2)-env.V(v,2)]; 
            env.H(vk) = env.H(vk)+kernel(ksize+d(1),ksize+d(2));
        end
      end
      
      % DEHEAT
      function deheat(env,v)
        env.H(logical(env.A(env.Vmap(v),:))) = 1;
      end
      
      % GET PATHS
      function getCycles(env)
        env.cycles = getAllCycles(env.Em,env.Vm);
        % Get Open Paths
        paths = splitCycles(env);
        env.cycles = [env.cycles 2*paths];
      end
      
      % RESET IDLENESS
      function IvReset(env)
        env.tl = zeros(1,length(env.V));
        env.Iv = zeros(1,length(env.V));
      end
      
      % UPDATE IDLENESS
      function IvUpdate(env,t,v)
        env.tl(env.Vmap(v)) = t;                    % Set the vertex as viewed
        env.tl(logical(env.A(env.Vmap(v),:))) = t;  % Set all adjacent vertexes as viewed
        % Update Idleness
        for i = 1:length(env.V)
            env.Iv(i) = t - env.tl(i);
        end
      end
      
      % PLOT BORDERS
      function plotBorders(env)
        plot(polyshape(env.B(:,1),env.B(:,2)),'FaceAlpha',0.2);
        hold on
        for n = 1:length(env.NFZ)
            plot(polyshape(env.NFZ{n}(:,1),env.NFZ{n}(:,2)),'FaceAlpha',0.2);
        end
        hold off
        axis equal
      end
      
      % PLOT GRAPHS
      function plotGraphs(env)
        plot(polyshape(env.B(:,1),env.B(:,2)),'FaceAlpha',0.2);
        hold on
        plot(graph(env.A),'XData',env.V(:,1),'YData',env.V(:,2));
        plot(graph(env.E(:,1),env.E(:,2),[],length(env.V)),'XData',env.V(:,1),'YData',env.V(:,2),'LineWidth', 2);
        for n=1:length(env.NFZ)
            plot(polyshape(env.NFZ{n}(:,1),env.NFZ{n}(:,2)),'FaceAlpha',0.2);
        end
        hold off
        axis equal
        legend('Environment','Vision','Movement')
      end
      
      % PLOT WEIGHT
      function plotWeight(env,res)
        [X,Y] = meshgrid(min(min(env.V)):1:max(max(env.V)));
        Z = ones(size(X));
        for x = 1:size(X,1)
            for y = 1:size(Y,1)
                z = ismembertol(env.V,[X(x,x),Y(y,y)],'ByRows',true);
                if sum(z) > 0
                    Z(x,y) = env.H(z);
                end
            end
        end
        Xq = min(min(env.V)):res:max(max(env.V)); 
        Yq = Xq;
        [Xq,Yq] = meshgrid(Xq,Yq);
        Zq = interp2(X,Y,Z,Xq,Yq,'spline');
        for x = 1:size(Xq,1)
            for y = 1:size(Yq,1)
                if ~inpolygon(Xq(x,x),Yq(y,y),env.B(:,1),env.B(:,2))
                    Zq(x,y) = NaN;
                end
            end
        end
        surf(Xq,Yq,Zq)
      end   

   end
end
