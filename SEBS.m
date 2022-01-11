% Copyright (c) 2020, Nicolò Bargellesi, Luca Facin & Lorenzo Marchini
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree.
%

%% Initial Conditions
%rng(1)      % Seed for reproducibility

load('Data/C.mat')      % load desired environment
env = C;                % rename selected environment
R = 2;                  % Number of Cameras
N = 130;                % Number of Iterations

% Camera Network
theta = 40;                         % AoV
e_tx = 0.05;                        % Transmission Error
v = randi(length(env.Vm),R,1);      % Cameras initial position
C = Camera.empty(R,0);
for r = 1:R
    C(r) = Camera(env,theta,v(r),0.25,0.05);
end

% SEBS parameters
L = 0.1;
M = 9*length(env.V)/R;

%% SEBS Simulation
X = zeros(R,N); % Paths
env.IvReset();
env.heatReset();
Hcheck = true;

for t = 1:N
    if t == 110
        %env.addHeat(2,1,1,10);              % Simulate event detection
    end
    X(:,t) = v;                             % Add the vertexes to the paths
    S = zeros(1,length(env.V));             % Initialize vertex intentions
    for r = randperm(R)
       v(r) = C(r).SEBS(env,R,S,L,M);       % Select next step
       % Share Intention
       view = env.A(env.Vmap(v(r)),:);
       view(1,env.Vmap(v(r))) = 1;
       % Simulate transmission errors
       if randi([0,100])/100 > e_tx
           S(logical(view)) = S(logical(view)) + 1;
       end
    end
    for r =1:R
       C(r).V = v(r);                       % Move
       % Simulate transmission errors
       if randi([0,100])/100 > e_tx
           env.IvUpdate(t,C(r).V)           % Update Idleness
           env.deheat(C(r).V)               % Deheat viewed vertices
       end
    end
    if t >= 110 && env.H(10)==1 && Hcheck
        Hcheck = false;
        fprintf('Zone Deheated in %f steps. \n', t-110);
    end
end
%% Plot Results
Xp = env.Vmap(X(:,103:end));
figure()
for i = 1:size(Xp,2)
    plot(polyshape(env.B(:,1),env.B(:,2)),'FaceAlpha',0.2);
    hold on
    for n=1:length(env.NFZ)
        plot(polyshape(env.NFZ{n}(:,1),env.NFZ{n}(:,2)),'FaceAlpha',0.2);
    end
    for r = 1:R
        scatter(env.V(Xp(r,i),1), env.V(Xp(r,i),2),'filled');     % plot position
        plot(env.V(Xp(r,1:i),1), env.V(Xp(r,1:i),2));             % plot path
        for k=1:i
            scatter(env.V(logical(env.A(Xp(r,k),:)),1), env.V(logical(env.A(Xp(r,k),:)),2), 'b');   % highlight viewed vertices
        end
    end
    hold off
    axis equal
    pause(0.5);
end
close 

%% Coverage Analysis (K1 tests of K2 coverages)
K1 = 1000;
K2 = 100;

Tvtot = zeros(1,K1);
Ivtot = zeros(1,K1); 
THcheck = zeros(1,K1);
for N = 1:K1
% Reset Camera Positions
v = randi(length(env.Vm),R,1);     % Cameras initial position
C = Camera.empty(R,0);
for r = 1:R
    C(r) = Camera(env,theta,v(r),0.25,0.05);
end
X = [v]; % Paths
env.IvReset();
env.heatReset();
Hcheck = true;
tHcheck = randi([1,round(10*length(env.Vm)/R)]);
n = 1;
t = 0;
Tv = zeros(1,K2);
Iv = zeros(1,K2);
viewed = zeros(1,size(env.V,1));
while n <= K2
    if t == tHcheck
        vHcheck = randi(length(env.V));
        env.addHeat(2,1,1,vHcheck);         % Simulate event detection
    end
    S = zeros(1,length(env.V));             % Initialize vertex intentions
    for r = randperm(R)
       v(r) = C(r).SEBS(env,R,S,L,M);       % Select next step
       % Share Intention
       view = env.A(env.Vmap(v(r)),:);
       view(1,env.Vmap(v(r))) = 1;
       S(logical(view)) = S(logical(view)) + 1; 
    end
    for r=1:R
       C(r).V = v(r);                       % Move
       if t > 100
        viewed = sign(viewed + env.A(env.Vmap(v(r)),:)); 
       end
       env.IvUpdate(t,C(r).V)               % Update Idleness
       env.deheat(C(r).V);                  % Deheat viewed vertices
    end
    X = [X v];                              % Add the vertexes to the paths
    if t >= tHcheck && env.H(vHcheck)==1 && Hcheck
        Hcheck = false;
        THcheck(N) = t-tHcheck; 
    end
    t = t + 1;
    Tv(n) = Tv(n) + 1;
    Iv(n) = Iv(n) + mean(env.Iv);
    if sum(viewed) == length(viewed)
        viewed = zeros(1,size(env.V,1));
        n = n + 1;
    end
end
    Tvtot(N) = mean(Tv(2:end))-1;
    Ivtot(N) = mean(Iv(2:end)./Tv(2:end));
end

fprintf('Mean Coverage Period: %2.3f \n', mean(Tvtot));
fprintf('Mean Idleness: %2.3f \n', mean(Ivtot));
fprintf('Mean Zone Deheated Period: %2.3f steps. \n', mean(THcheck));

%% Percentage Coverage Analysis (K1 tests at K2 steps)
K1 = 1000;
K2 = 12;

ViewTot = zeros(1,K1); 
for N = 1:K1
% Reset Camera Positions
v = randi(length(env.Vm),R,1);     % Cameras initial position
C = Camera.empty(R,0);
for r = 1:R
    C(r) = Camera(env,theta,v(r),0.25,0.05);
end
X = []; % Paths
env.IvReset();
env.heatReset();
K = inf;
n = 1;
t = 0;
viewed = zeros(1,size(env.V,1));
while n <= K
    X = [X v];                              % Add the vertexes to the paths
    S = zeros(1,length(env.V));             % Initialize vertex intentions
    for r = randperm(R)
       v(r) = C(r).SEBS(env,R,S,L,M);       % Select next step
       % Share Intention
       view = env.A(env.Vmap(v(r)),:);
       view(1,env.Vmap(v(r))) = 1;
       S(logical(view)) = S(logical(view)) + 1; 
    end
    for r=1:R
       C(r).V = v(r);                       % Move
       viewed = sign(viewed + env.A(env.Vmap(v(r)),:)); 
       env.IvUpdate(t,C(r).V)               % Update Idleness
       env.deheat(C(r).V);                  % Deheat viewed vertices
    end
    t = t + 1;
    n = n + 1;
    if sum(viewed) == length(viewed) && K==inf
        viewed = zeros(1,size(env.V,1));
        if t>100
            K = n+K2;
        end
    end
end
    ViewTot(N) = sum(viewed)./length(env.V);
end
fprintf('Mean Coverage Percentage: %2.3f \n', mean(ViewTot));
