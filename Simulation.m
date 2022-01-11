% Copyright (c) 2019, Nicolò Bargellesi, Luca Facin & Lorenzo Marchini
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree.
%

clear
close all
clc


%% Coordinated Surveillance
N = 200;                    % duration
Npre = 500;                 % duration transient
Simulation_preprocessing;   % load data

% SEBS Initialization
Np = NoC;
s1 = round(0.5/Tc);
env.IvReset();
env.heatReset();
d = zeros(NoC,2);
detected = false;
for t = 1:Npre
    S = zeros(1,length(env.V));             % Initialize vertex intentions
    for n = randperm(NoC)
        % SEBS
        if t/s1 == round(t/s1)
            v(n) = C(n).SEBS(env,Np,S,L,M);       % Select next step
            d(n,:) = env.Vm(v(n),:) - C(n).X(1:2);
            % Share Intention
            view = env.A(env.Vmap(v(n)),:);
            view(1,env.Vmap(v(n))) = 1;
            % Simulate transmission errors
            if randi([0,100])/100 > e_tx
                S(logical(view)) = S(logical(view)) + 1;
            end
        end
    end
    % MOVE
    for n = 1:NoC
       C(n).V = v(n);                     
       C(n).X(1:2) = C(n).X(1:2) + d(n,:)/s1;
       % Simulate transmission errors
       if randi([0,100])/100 > e_tx
            env.IvUpdate(t*Tc,C(n).V)        % Update Idleness
            env.deheat(C(n).V)               % Deheat viewed vertices
       end
    end
end

% Tracking Control Initialization
nL = 0;
s = round(T/Tc);
realZ = [];
hctrl.h0 = C(1).zMax;
hctrl.vh0 = 0;

%   open Simulink model
open_system('control.slx');

figure()
for t = Npre+1:N+Npre
    env.plotBorders();
    hold on
    
    % TARGET TRAJECTORY
    if t > Npre+50
        x = (Ac*x' + Qc*randn(4,1))'; % New Trajectory point
        % verify if new trajectory point is inside our environment
        in = inpolygon(x(1),x(2),B(:,1),B(:,2));
        in1 = inpolygon(x(1),x(2),NFZ2{1}(:,1),NFZ2{1}(:,2));
        in2 = inpolygon(x(1),x(2),NFZ2{2}(:,1),NFZ2{2}(:,2));
        if in
            scatter(x(1),x(2),30,[0.8500, 0.3250, 0.0980],'filled')
        end
    % DETECTION
    firstStep = false;
    for n = 1:NoC
        if (abs(C(n).X(1)-x(1)) < C(n).FoV && abs(C(n).X(2)-x(2)) < C(n).FoV && ~detected && in && ~in1 && ~in2)
            C(n).Task = 1;
            Np = Np-1;
            env.addHeat(3,1,1,C(n).V);             
            detected = true;
            firstStep = true;
            tDetect = t;
            % Initialize Filter
            r = C(n).FoV*C(n).eFoV;                   
            R = r^2*eye(2);                           
            y = (H*x' + sqrt(R)*randn(2,1))';
            x_pred = [y,0 0];
            P_pred = blkdiag(R,eye(2));
            d(n,:) = x_pred(1:2) - C(n).X(1:2);
            break
        end
    end
    end
    
    
    S = zeros(1,length(env.V));             % Initialize vertex intentions
    for n = randperm(NoC)
        % SEBS
        if t/s1 == round(t/s1) && C(n).Task == 0
            v(n) = C(n).SEBS(env,Np,S,L,M);       % Select next step
            d(n,:) = env.Vm(v(n),:) - C(n).X(1:2);
            % Share Intention
            view = env.A(env.Vmap(v(n)),:);
            view(1,env.Vmap(v(n))) = 1;
            S(logical(view)) = S(logical(view)) + 1;
        elseif t/s == round(t/s) && C(n).Task == 1 && ~firstStep
            if randi([0,100])/100 > err && abs(C(n).X(1)-x(1)) < C(n).FoV && abs(C(n).X(2)-x(2)) < C(n).FoV
                r = C(n).FoV*C(n).eFoV;                   
                R = r^2*eye(2);                           
                y = (H*x' + sqrt(R)*randn(2,1))';
                nL = 0;
            else
                y = [0 0];  % loss of a measure
                nL = nL+1;
            end
            % Predicton
            [x_pred, P_pred] = kalman(A,H,Q,R,y,x_pred,P_pred);
            d(n,:) = x_pred(1:2) - C(n).X(1:2);
            sd = sqrt(P_pred);
            sdMax = max((sd(1,1)+sd(2,2))/2 , T*(sd(3,3)+sd(4,4))/2);
            % TRACKING LOSS
            % verify if the prediction is inside the allowed area
            in = inpolygon(x_pred(1),x_pred(2),B(:,1),B(:,2));
            in1 = inpolygon(x_pred(1),x_pred(2),NFZ2{1}(:,1),NFZ2{1}(:,2));
            in2 = inpolygon(x_pred(1),x_pred(2),NFZ2{2}(:,1),NFZ2{2}(:,2));
            if ~in || in1 || in2 || nL>5
                C(n).Task = 0;
                Np = Np+1;
                index = find(ismembertol(env.V,round(C(n).X(1:2)),'ByRows',true));
                env.addHeat(3,1,1,index);
                index1 = logical(env.Vmap==index);
                if sum(index1) == 0
                    index = find(env.A(index,:));
                    v(n) = find(env.Vmap==index(1));
                    d(n,:) = env.Vm(v(n),:) - C(n).X(1:2);
                else
                    v(n) = find(index1);
                    d(n,:) = env.Vm(v(n),:) - C(n).X(1:2);
                end
                detected = false;
            end
        end
    end

    
    % MOVE
    for n = 1:NoC
        if C(n).Task == 0
            C(n).V = v(n);                     
            C(n).X(1:2) = C(n).X(1:2) + d(n,:)/s1;
            if C(n).X(3) - C(n).zMax > 0.01
                z = C(n).zMax;
                sim('control');
                hctrl.h0 = symout.data(end,2);
                hctrl.vh0 = symout.data(end,1);
                C(n).updateZ(hctrl.h0);
            else
                C(n).updateZ(C(n).zMax);
            end
        elseif C(n).Task == 1
            C(n).X(1:2) = C(n).X(1:2) + d(n,:)/s;
            % ZOOM
            if t-tDetect >= Ts*s
                [z,k] = C(n).optimalZoom(gamma,sdMax);
                zoom = [z;k];
                sim('control');
                hctrl.h0 = symout.data(end,2);
                hctrl.vh0 = symout.data(end,1);
                C(n).updateZ(hctrl.h0);
            end
        end
       C(n).plot;
       env.IvUpdate(t*Tc,C(n).V)            % Update Idleness
       env.deheat(C(n).V)                   % Deheat viewed vertices
    end
    
    hold off
    pause(Tc)
end
