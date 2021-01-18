clear
close all
clc
%% Evironment

N = 500;

B = [0 0; 0 9; 7 9; 7 6; 3 6; 3 3; 7 3; 7 0];       % C Environment
%B = [0 0; 0 7; 4 7; 4 3; 6 3; 6 0];                % L Environment

res = 1;

env = Environment(B,{});
env.cellularize(res);

%% Generate Camera
Ts = 5;             % setteling time for P before starting zoom
theta = 40;         % angle (1m^2 from 2m)
zmin = 0.25;        % minimum height
gamma = 12;         % trade off parameter
eFoV = 0.15;        % percentage error of view
err = 0.00;         % measure loss percentage

cam = Camera(env,theta,10,zmin,eFoV);

%% Starting Point of Real Trajcetory
j=1;
while j<=N

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
v = bar-pos;
% verify if starting is speed inwards the environment
in = inpolygon(pos(1)+v(1)/100,pos(2)+v(2)/100,B(:,1),B(:,2));
if ~in
    v = -v;
end

% Real Trajectory model
a = 1;                  % speed [m/s]
Tc = 0.1;               % sampling period [s]
v = a*(v)/norm(v);      % speed vector [vx,vy]
x = [pos,v];            % state vector
q = 0.15;                 % process noise standard deviation

% process noise covariance matrix
Qc = q*[0 0 0 0;0 0 0 0;0 0 1 0;0 0 0 1]; % random speed (q = 0.x)
% random acceleration (q = x)
%Qc = q*[Tc^4/4,0,Tc^3/2,0;0,Tc^4/4,0,Tc^3/2;Tc^3/2,0,Tc^2,0;0,Tc^3/2,0,Tc^2];

% transition matrix  
Ac = [1,0,Tc,0;0,1,0,Tc;0,0,1,0;0,0,0,1]; 

% Filter trajectory model
T = 0.3;       % sampling period of Kalman filter
q = 10;         % process noise standard deviation

% process noise covariance matrix (random acceleration)
Q = q*[T^4/4,0,T^3/2,0;0,T^4/4,0,T^3/2;T^3/2,0,T^2,0;0,T^3/2,0,T^2];
% transition matrix  
A = [1,0,T,0;0,1,0,T;0,0,1,0;0,0,0,1]; 

% Filter Measurement model
H = [1 0 0 0; 0 1 0 0];     % measurement matrix
r = cam.FoV*cam.eFoV;       % measurement noise standard deviation
R = r^2*eye(2);               % measurement noise covariance matrix

% One Step Ahead Prediction
s = round(T/Tc);
out = false;

% Filter Initialization 
% NOTE it must be set in the point where the first detection has been done
y = (H*x(1,:)' + sqrt(R)*randn(2,1))';
x_pred = [y(1,:),0 0];
x_pred(1+s,:) = [y(1,:),0 0];
P_pred = blkdiag(R,eye(2));
P_pred(:,:,1+s) = blkdiag(R,eye(2));
sd = sqrt(P_pred(:,:,1));
sdMax = max((sd(1,1)+sd(2,2))/2 , T*(sd(3,3)+sd(4,4))/2);
sdMax(1+s) = max((sd(1,1)+sd(2,2))/2 , T*(sd(3,3)+sd(4,4))/2);
% zoom = [cam.zMax; 3].*ones(2,(Ts*s)+1);
zoom = [cam.zMax; cam.FoV/sdMax(1)];
zoom(:,1+s) = [cam.zMax; cam.FoV/sdMax(1+s)];

for i = 2:s
    % New Trajectory point
    x = [x; (Ac*x(i-1,:)' + Qc*randn(4,1))'];
    % verify if new trajectory point is inside our environment
    in = inpolygon(x(i,1),x(i,2),B(:,1),B(:,2));
    if ~in
        out = true;
    end
end

count = s;
if ~out
    i = s+1;
else
    i=0;
end
while i ~= 0
    % New Trajectory point
    x = [x; (Ac*x(i-1,:)' + Qc*randn(4,1))'];
    
    if count >= s
        count = 1;
        % Detect
        if randi([0,100])/100 > err
          r = cam.FoV*cam.eFoV;                   
          R = r^2*eye(2);                           
          y(i,:) = (H*x(i,:)' + sqrt(R)*randn(2,1))';
        else
          y(i,:) = [0 0];  % loss of a measure
        end
        % Predicton
        [x_pred(i+s,:), P_pred(:,:,i+s)] = kalman(A,H,Q,R,y(i,:),x_pred(i,:),P_pred(:,:,i));
        sd = sqrt(P_pred(:,:,i+s));
        sdMax(i+s) = max((sd(1,1)+sd(2,2))/2 , T*(sd(3,3)+sd(4,4))/2);
        % Zoom Control
        if i >= Ts*s
          [z,k] = cam.optimalZoom(gamma,sdMax(i+s));
          zoom(:,i+s) = [z;k];
          cam.updateZ(zoom(1,i+s));
        else
          zoom(:,i+s) = [cam.zMax; cam.FoV/sdMax(i+s)];
        end
    else
        count = count + 1;
    end
    
    % verify if new trajectory point is inside our environment
    in = inpolygon(x(i,1),x(i,2),B(:,1),B(:,2));
    if ~in
        i=0;
    else
        i = i+1;
    end
end

% Correct Indices
for t1 = 1:s:size(y,1)
    x_pred(t1+1:t1+s-1,:) = x_pred(t1+s,:).*ones(s-1,4);
    P_pred(:,:,t1+1:t1+s-1) = P_pred(:,:,t1+s).*ones(4,4,s-1);
    y(t1+1:t1+s-1,:) = y(t1,:).*ones(s-1,2);
end

% Tracking performance index
if size(x,1)>Ts*s
pred = x_pred((Ts*s):s:size(x,1),1:2);
real = x((Ts*s):s:size(x,1),1:2);
track = sqrt((real(:,1)-pred(:,1)).^2+(real(:,2)-pred(:,2)).^2);
trk_err(j) = mean(track);
trk_errMax(j) = max(track);

j=j+1;
end
 
end

fprintf('Mean Traking Error: %2.3f \n', mean(trk_err));
fprintf('Max Traking Error: %2.3f \n', mean(trk_errMax));
fprintf('MaxM Traking Error: %2.3f \n', max(trk_errMax));
%% Plot

% figure()
% for t = 1:size(x,1)
%     env.plotBorders()
%     hold on
%     scatter(x(t,1),x(t,2),30,[0.8500, 0.3250, 0.0980],'filled')
%     plot(x(1:t,1),x(1:t,2),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980])
%     plot(y(1:t,1),y(1:t,2),'o','MarkerSize',3,'Color',[0, 0.4470, 0.7410])
%     plot(x_pred(1:t,1),x_pred(1:t,2),'LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250])
%     % Plot Camera
%     cam.X(1) = x_pred(t,1);
%     cam.X(2) = x_pred(t,2);
%     cam.updateZ(zoom(1,t));
%     cam.plot()
%     hold off
%     axis equal
%     pause(Tc)
% end
% 
% % Uncertainty
% beta = 0.6161;
% u = @(k) (1-exp(-k/beta));
% % Information Lost (due to distance)
% I = @(z) -(exp(z-cam.zmin)-1);
% 
% figure()
% subplot(3,1,1)
% plot(zoom(1,1:s:end))
% ylim([0,cam.zMax+0.1])
% subplot(3,1,2)
% plot(abs(I(zoom(1,1:s:end))))
% subplot(3,1,3)
% plot(u(zoom(2,1:s:end)))

%% Zoom Analysis
% figure()
% for t = 1:s:size(x,1)
%     % Plot Camera
%     cam.X(1) = x_pred(t,1);
%     cam.X(2) = x_pred(t,2);
%     cam.updateZ(zoom(1,t));
%     scatter(0,0,30,[0.9290, 0.6940, 0.1250],'filled')
%     hold on
%     rectangle('Position',[-cam.FoV -cam.FoV 2*cam.FoV 2*cam.FoV])
%     scatter(x(t,1)-cam.X(1),x(t,2)-cam.X(2),30,[0.8500, 0.3250, 0.0980],'filled')
%     rectangle('Position',[-sdMax(t) -sdMax(t) 2*sdMax(t) 2*sdMax(t)],'Curvature',1)
%     rectangle('Position',[-3*sdMax(t) -3*sdMax(t) 2*3*sdMax(t) 2*3*sdMax(t)],'Curvature',1)
%     rectangle('Position',[-2*sdMax(t) -2*sdMax(t) 2*2*sdMax(t) 2*2*sdMax(t)],'Curvature',1)
%     hold off
%     axis equal
%     xlim([-env.Res-0.1 env.Res+0.1])    
%     ylim([-env.Res-0.1 env.Res+0.1])
%     %pause()
% end