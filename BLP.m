% Copyright (c) 2020, Nicolò Bargellesi, Luca Facin & Lorenzo Marchini
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree.
%

%% Parameters
load('Data/C.mat')      % load desired environment
env = C;                % rename selected environment
R = 2;                  % # of cameras
% lambda = 0.0001;      % trade-off parameter (old suboptimal problem)
% lambda = 0.5;         % trade-off parameter (old optimal problem)

%% Cycles Viewed Area
Av = env.A;
FoV = 1;
for k=2:FoV
    Av = sign(Av+(Av*env.A));
end

Vv = zeros(length(env.V),size(env.cycles,2));
for i = 1:size(env.cycles,2)
    verts = unique([env.E(logical(env.cycles(:,i)),1); env.E(logical(env.cycles(:,i)),2)]);
    Vv(:,i) = sign(sum(Av(verts,:)))';
end

%% Weighted Length
env.heatReset();
%env.addHeat(2,1,1,19);              % Simulate event detection

% Cycles Weights
cW = zeros(1,size(env.cycles,2));
for i = 1:size(env.cycles,2)
    k = sum(env.cycles(:,i))/sum(logical(env.cycles(:,i)));
    verts = unique([env.Em(logical(env.cycles(:,i)),1); env.Em(logical(env.cycles(:,i)),2)]);
    
    cW(i) = k*sum(env.H(env.Vmap(verts)));
    if k == 2
        cW(i) = cW(i)-2;
    end
end

%% Optimization Problem
prob = optimproblem;
% Variables (y for old optimal problem)
x = optimvar('x',size(env.cycles,2),'Type','integer','LowerBound',0,'UpperBound',1);
% y = optimvar('y',size(comb,1),'Type','integer','LowerBound',0,'UpperBound',1);    

% Usefull Values
% l = sum(env.cycles);                          % length of cycles (old problem)
l = exp(cW/min(cW))/min(exp(cW/min(cW)));       % normalized exponential weight of cycles (for max length accent)
xV = [];
for i = 1:length(env.V)
    xV = [xV; x'];
end


% Objective Function
% Optimal (Fast)
prob.Objective = sum(x'.*l);
% Suboptimal (Old)
%prob.Objective = sum(x'.*l)+lambda*sum((x(env.combC(:,1))'+x(env.combC(:,2))').* abs(cW(env.combC(:,1)) - cW(env.combC(:,2))));
% Optimal (Old)
% prob.Objective = sum(x'.*l)+lambda*sum(((x(comb(:,1))'+x(comb(:,2))'-1)+y')/2.* abs(cW(comb(:,1)) - cW(comb(:,2))));

% Problem Constraints
cons1 = sum(x) <= R;                            % # of Cameras
cons2 = sum(xV.*Vv,2) >= sign(sum(env.A,2));    % full coverage
prob.Constraints.cons1 = cons1;
prob.Constraints.cons2 = cons2;
% Absolute Value Constraints (old problem)
% abs1 = +(x(comb(:,1))'+x(comb(:,2))'-1) - y' <= 0;
% abs2 = -(x(comb(:,1))'+x(comb(:,2))'-1) - y' <= 0;
% prob.Constraints.abs1 = abs1;
% prob.Constraints.abs2 = abs2;

% show(prob)

%% Solve Problem
sol = solve(prob);
sel = find(sol.x);

%% Optimal Paths
optCycles = env.cycles(:,sel);

optE = [];
for n=1:size(optCycles,2)
    optE = [optE; env.Em(logical(optCycles(:,n)),:)];
end

% Translate in environment domain
optE = remapEdges(optE,env.Vmap);

% Plot Optimal Cycles
figure(2)
plot(polyshape(env.B(:,1),env.B(:,2)),'FaceAlpha',0.2);
hold on
for n=1:length(env.NFZ)
    plot(polyshape(env.NFZ{n}(:,1),env.NFZ{n}(:,2)),'FaceAlpha',0.2);
end
plot(graph(optE(:,1),optE(:,2),[],length(env.V)),'XData',env.V(:,1),'YData',env.V(:,2),'LineWidth', 2);
hold off
axis equal

%% Coverage Analysis
Tv = max(sum(optCycles))-1;

V = {};
for r = 1:R
    e = find(optCycles(:,r));
    v = zeros(1,length(e)+1);
    for j = 1:length(e)
        if j == 1
            v(j) = env.Em(e(j),1);
            v(j+1) = env.Em(e(j),2);
        else
            A1 = env.Em(e,1) == v(j);
            A2 = env.Em(e,2) == v(j);
            A = logical(A1+A2);
            B = [env.Em(e(A),1); env.Em(e(A),2)];
            C1 = B ~= v(j);
            C2 = B ~= v(j-1);
            C = logical(C1.*C2);
            v(j+1) = B(C);
         end
    end
    V{r} = v;
end


Iv = 0;
env.IvReset();
for t = 1:2*Tv
    for r = 1:R
        if length(V{r}) < t
            if max(optCycles(:,r)) > 1
                V{r} = [V{r} fliplr(V{r}(1:end-1))];
            else
                V{r} = [V{r} V{r}(2:end)];
            end
        end
        env.IvUpdate(t,V{r}(t));      % Update Idleness
    end
    if t>Tv
        Iv = Iv + mean(env.Iv);
    end
end


fprintf('Coverage Period: %2.3f \n', Tv);
fprintf('Mean Idleness: %2.3f \n', Iv/Tv);
