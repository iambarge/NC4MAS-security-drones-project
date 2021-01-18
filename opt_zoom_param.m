%% Parameters 
load('Data/C.mat')      % load desired environment
env = C;                % rename selected environment
Ts = 5;                 % setteling time for P before starting zoom
theta = 40;             % angle (1m^2 from 2m)
gamma = 12;             % trade off parameter
eFoV = 0.05;            % percentage error of view
err = 0.05;             % measure loss percentage
sigma_P = 0.25;         % kalman prediction error sd

% Generate Camera
cam = Camera(env,theta,1,0,eFoV);

%% Coverage Factor Analysis 
tx = 0:0.00001:1;               % Real Uncertainty
ts = tinv((tx+1)/2,inf);        % Real Coverage factors

% Approximated Uncertainty
u = @(k,beta) (1-exp(-k/beta)); 

% Interest Zone
lb = logical(ts >= 2);
hb = logical(ts <= 3);
bound = logical(lb.*hb);

% Cost Function
u0 = @(beta) norm((tx(bound)-u(ts(bound),beta)));

% Solve
beta = fmincon(u0,0.5,[],[],[],[],0,1);

figure()
plot(ts,tx)
hold on
plot(ts,u(ts,beta))
xlim([0 3.5])

%% Trade-off Parameter Analysis

% Gamma Test
y = [];
t = 1:5:100;
for g = t
[z,k] = cam.optimalZoom(g,sigma_P);
x = [z;k];
y = [y x];
end

figure()
plot(t,y(1,:))
hold on
plot(t,y(2,:))

% sigma_P Test
y = [];
t = 0.05:0.005:0.55;
for sigma_p = t
[z,k] = cam.optimalZoom(gamma,sigma_p);
x = [z;k];
y = [y x];
end

figure()
plot(t,y(1,:))
hold on
plot(t,y(2,:))
