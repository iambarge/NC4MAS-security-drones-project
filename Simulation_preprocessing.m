%% Define Environment
% Borders
B1 = [0 0; 0 9; 7 9; 7 6; 3 6; 3 3; 7 3; 7 0];      % C Environment
B2 = [0 0; 0 7; 4 7; 4 3; 6 3; 6 0];                % L Environment 
B3 = [0 0; 0 6; 5 6; 5 0];                          % Rectangular Environment (rect_XxY)
% Large Scale Environment
B4 = [0 0; 0 9; 2 9; 2 14; 14 14; 14 0; 9 0; 9 1; 3 1; 3 4; 2 4; 2 0];

B = B4;

% No Fly Zones
NFZ1 = {};
NFZ2 = {[6 4; 7 4; 7 5; 6 5],[10 6; 12 6; 12 11; 5 11; 5 8; 10 8]};

% Select Environment            
env = Environment(B,NFZ2);

figure(1)
env.plotBorders();

%% Cellularize Environment
res = 1;

env.cellularize(res);

% Plot Graphs
figure(1)
env.plotGraphs();

%% Detect Cycles (Slow)
%tic
%env.getCycles();
%toc

% Print Cycles
% for n=1:size(env.cycles,2)
%     grPlot(V,E(find(env.cycles(:,n)),:),'g','%d',''); % one cycle
%     title(['\bfCycle N' num2str(n)]);
%     close % ADD BREAKPOINT
% end

%% Camera Network
NoC = 6;                            % Number of Cameras
Ts = 5;                             % setteling time for P before starting zoom
theta = 40;                         % angle (1m^2 from 2m)
gamma = 12;                         % trade off parameter
eFoV = 0.05;                        % percentage error of view
err = 0.05;                         % measure loss percentage
e_tx = 0.05;                        % transmission error
v = randi(length(env.Vm),NoC,1);    % Cameras initial position

C = Camera.empty(NoC,0);
for r = 1:NoC
    C(r) = Camera(env,theta,v(r),1,eFoV);
end

% SEBS parameters
L = 0.1;
M = 9*length(env.V)/NoC;

%% Starting Point of Real Trajcetory
pos = randi([min(min(B)),max(max(B))],1,2);
bar = [(min(B(:,1))+max(B(:,1)))/2, (min(B(:,2))+max(B(:,2)))/2];
% verify if starting point is on environment borders
[~,on] = inpolygon(pos(1),pos(2),B(:,1),B(:,2));
if ~on
    while ~on
        pos = randi([min(min(B)),max(max(B))],1,2);
        [~,on] = inpolygon(pos(1),pos(2),B(:,1),B(:,2));
    end
end
vT = bar-pos;
% verify if starting is speed inwards the environment
in = inpolygon(pos(1)+vT(1)/100,pos(2)+vT(2)/100,B(:,1),B(:,2));
if ~in
    vT = -vT;
end

%% Real Trajectory model
a = 1;                  % speed [m/s]
Tc = 0.1;               % sampling period [s]
vT = a*(vT)/norm(vT);   % speed vector [vx,vy]
x = [pos,vT];           % state vector
q = 15;                 % process noise standard deviation

% process noise covariance matrix
%Qc = q*[0 0 0 0;0 0 0 0;0 0 1 0;0 0 0 1]; % random speed (q = 0.x)
% random acceleration (q = x)
Qc = q*[Tc^4/4,0,Tc^3/2,0;0,Tc^4/4,0,Tc^3/2;Tc^3/2,0,Tc^2,0;0,Tc^3/2,0,Tc^2];

% transition matrix  
Ac = [1,0,Tc,0;0,1,0,Tc;0,0,1,0;0,0,0,1]; 

%% Filter trajectory model
T = 0.2;        % sampling period of Kalman filter
q = 10;         % process noise standard deviation

% process noise covariance matrix (random acceleration)
Q = q*[T^4/4,0,T^3/2,0;0,T^4/4,0,T^3/2;T^3/2,0,T^2,0;0,T^3/2,0,T^2];

% transition matrix  
A = [1,0,T,0;0,1,0,T;0,0,1,0;0,0,0,1]; 

% measurement model
H = [1 0 0 0; 0 1 0 0]; 

%% Hight Controller Sinthesys
hctrl.A = [0 1; 1 0];
hctrl.B = [1; 0];
hctrl.C = [1 0; 0 1];
hctrl.D = [0; 0];
hctrl.C1 = [0 1];
hctrl.D1 = 0;

hctrl.sys = ss(hctrl.A,hctrl.B,hctrl.C1,hctrl.D1);

%   position controller specs
hctrl.ts = T;       %   desired settling time (at 5%)
hctrl.Mp = 0.01;    %   desired overshoot

%   get specs for loop tf 
hctrl.d = log(1/hctrl.Mp) / sqrt(pi^2 + log(1/hctrl.Mp)^2);                   %   damping factor
hctrl.wgc = 3/hctrl.d/hctrl.ts;                                               %   gain crossover freq [rad/s]                                 
hctrl.phim = 180/pi * atan(2*hctrl.d/sqrt(sqrt(1+4*hctrl.d^4)-2*hctrl.d^2));  %   phase margin [deg]

%   position controller data
hctrl.alpha = 4;

%   filtered derivative tf
hctrl.wc   = 10*hctrl.wgc; 
hctrl.numD = [hctrl.wc, 0];
hctrl.denD = [1, hctrl.wc];

%   control design
[hctrl.kp, hctrl.ki, hctrl.kd] = get_PID(hctrl.sys, hctrl.wgc, hctrl.phim, hctrl.alpha);

% Anti Windup Gain
hctrl.kw = 1/hctrl.ts;
