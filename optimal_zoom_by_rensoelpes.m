%% Parameters 
Ts = 5;             % setteling time for P before starting zoom
theta = 40;         % angle (1m^2 from 2m)
zmin = cam.zMax/4;        % minimum height
gamma = 12;         % trade off parameter
eFoV = 0.05;        % percentage error of view
err = 0.05;         % measure loss percentage

gamma = 12;
P = 0.25;

% Generate Camera
cam = Camera(env,theta,1,zmin,eFoV);

%% Coverage Factor Analysys 
%********* TRIAL AND ERROR *****************
beta = 0.62;
u = @(k) (1-exp(-k/beta));

tx = 0:0.001:1;
ts = tinv((tx+1)/2,inf);
lb = logical(ts >= 2);
hb = logical(ts <= 3);
bound = logical(lb.*hb);
mse = norm(tx(bound)-u(ts(bound)));

figure()
plot(ts,tx)
hold on
plot(ts,u(ts))
%xlim([2 3])

%% Coverage Factor Analysys
%********OPTIMIZATION*********** 
tx = 0:0.00001:1;
ts = tinv((tx+1)/2,inf);
lb = logical(ts >= 2);
hb = logical(ts <= 3);
bound = logical(lb.*hb);
u = @(k,beta) (1-exp(-k/beta));
% u0 = @(beta) max(abs(tx(bound)-u(ts(bound),beta)));
u0 = @(beta) norm((tx(bound)-u(ts(bound),beta)));
% Optimal beta
beta = fmincon(u0,0.5,[],[],[],[],0,1);

%% Graph
figure()
plot(ts,tx,'LineWidth',1.5)
hold on
plot(ts,u(ts,beta),'LineWidth',1.5)
fill([2 3 3 2 2],[-0.1 -0.1 1.2 1.2 -0.1],'b','facealpha',.1,'LineStyle','none')
%xline(2,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
%xline(3,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
grid(gca,'minor')
grid on
ax = gca;
ax.YLim = [-0.1 1.15];
ax.XLim = [0 3.5];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.XLabel.Interpreter = 'latex';
ax.XLabel.String = '$k$';
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$u(k)$';
legend({'$u(k)=2\Phi(k)-1$','$u(k)=1-e^{-{k \over b}}$','Domain of $u(k)$'},'Location','northwest'); 
ax.Legend.Interpreter = 'latex'; 
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';

%%
figure()
plot(ts,tx,'LineWidth',1.5)
fill([2 3 3 2 2],[-0.1 -0.1 1.2 1.2 -0.1],'b','facealpha',.1,'LineStyle','none')
%xline(2,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
%xline(3,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
grid(gca,'minor')
grid on
ax = gca;
ax.YLim = [-0.1 1.1];
ax.XLim = [0 3.5];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.XLabel.Interpreter = 'latex';
ax.XLabel.String = '$k$';
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$u(k)$';
legend({'$u(k)=2\Phi(k)-1$','$u(k)=1-e^{-{k \over b}}$'},'Location','northwest'); 
ax.Legend.Interpreter = 'latex'; 
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';



%% Optimization Problem

% Information Lost (due to distance)
I = @(z) -(exp((z-cam.zmin))-1);

%f_old = @(z,k) -(exp(-(z-zmin))+gamma*u(k));
f = @(z,k) -(I(z) + gamma*u(k));
g = @(z,k) (k*sqrt(P)/tan(theta))-z;

% figure()
% fsurf(f,[cam.zmin cam.zMax 2 3])
% hold on
% fsurf(g,[cam.zmin cam.zMax 2 3])


% Solve to obtain gamma
y = [];
t = 0.05:0.005:0.55;
for P = t
[z,k] = cam.optimalZoom(12,P);
x = [z;k];
y = [y x];
end

%% 
% Solve to obtain the trade off 
P1 = 0.13;
y1 = [];
t = 1:50;
for gamma = t
[z,k] = cam.optimalZoom(gamma,P1);
x1 = [z;k];
y1 = [y1 x1];
end

P2 = 0.19;
y2 = [];
t = 1:50;
for gamma = t
[z,k] = cam.optimalZoom(gamma,P2);
x2 = [z;k];
y2 = [y2 x2];
end

P3 = 0.25;
y3 = [];
t = 1:50;
for gamma = t
[z,k] = cam.optimalZoom(gamma,P3);
x3 = [z;k];
y3 = [y3 x3];
end

%%
tDown= 0.05:0.005:0.085;
kDown= (y(1,1:8)*tan(cam.Theta*pi/180))./tDown;
tUp = 0.50:0.01:0.55;
kUp= (cam.zMax*tan(cam.Theta*pi/180))./tUp;

figure()
subplot(2,1,1)
plot(t,y(1,:),'LineWidth',1)
hold on;

yline(zmin,'--','LineWidth',.8,'Color',[0, 0.4470, 0.7410]);
yline(cam.zMax,'--','LineWidth',.8,'Color',[0, 0.4470, 0.7410]);
xline(0.5,'--','LineWidth',.8,'Color','k');
xline(0.085,'--','LineWidth',.8,'Color','k');
grid on
ax = gca;
ax.YLim = [0.1 1.3];
ax.XLim = [0.05 0.55];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$z^*$';


subplot(2,1,2)
plot(t,y(2,:),'LineWidth',1,'color',[0.8500, 0.3250, 0.0980])
hold on;
plot(tDown,kDown,'LineWidth',1,'color',[0.9290, 0.6940, 0.1250])
plot(tUp,kUp,'LineWidth',1,'color',[0.9290, 0.6940, 0.1250])
yline(2,'--','LineWidth',.8,'Color',[0.8500, 0.3250, 0.0980]);
yline(3,'--','LineWidth',.8,'Color',[0.8500, 0.3250, 0.0980]);
xline(0.5,'--','LineWidth',.8,'Color','k');
xline(0.085,'--','LineWidth',.8,'Color','k');
grid on
ax = gca;
%ax.YLim = [1.75 3.25];
ax.XLim = [0.05 0.55];
ax.XLabel.FontSize = 15;
ax.YLabel.FontSize = 15;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$k^*$';
ax.XLabel.Interpreter = 'latex';
ax.XLabel.String = '$\sigma_p$';

%%
% figure()
% plot(t,y(1,:),'LineWidth',1.5)
% hold on;
% plot(t,y(2,:),'LineWidth',1.5)
% yline(zmin,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% yline(cam.zMax,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% grid(gca,'minor')
% grid on
% ax = gca;
% ax.YLim = [0 3.2];
% ax.XLim = [1 100];
% ax.XLabel.FontSize = 15;
% ax.YLabel.FontSize = 15;
% ax.XLabel.Interpreter = 'latex';
% ax.XLabel.String = '$\gamma$';
% legend({'$z$','$k$','Limits'},'Location','best'); 
% ax.Legend.Interpreter = 'latex'; 
% ax.Legend.FontSize = 10;
% ax.Legend.Box = 'on';


figure()
subplot(2,1,1)
plot(t,y1(1,:),'LineWidth',1)
hold on;
plot(t,y2(1,:),'LineWidth',1)
plot(t,y3(1,:),'LineWidth',1)
yline(zmin,'--','LineWidth',1,'Color','k');
yline(cam.zMax,'--','LineWidth',1,'Color','k');
%grid(gca,'minor')
grid on
ax = gca;
ax.YLim = [0.1 1.3];
ax.XLim = [1 50];
ax.XLabel.FontSize = 18;
ax.YLabel.FontSize = 18;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$z^\star$';
legend({'$\sigma_p = 0.13$','$\sigma_p=0.19$','$\sigma_p=0.25$'},'Location','best'); 
ax.Legend.Interpreter = 'latex'; 
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';

subplot(2,1,2)
plot(t,y1(2,:),'LineWidth',1)
hold on;
plot(t,y2(2,:),'LineWidth',1)
plot(t,y3(2,:),'LineWidth',1)

%grid(gca,'minor')
grid on
ax = gca;
ax.YLim = [1.75 3.25];
ax.XLim = [1 50];
ax.XLabel.FontSize = 18;
ax.YLabel.FontSize = 18;
ax.YLabel.Interpreter = 'latex';
ax.YLabel.String = '$k^\star$';
ax.XLabel.Interpreter = 'latex';
ax.XLabel.String = '$\gamma$';

legend({'$\sigma_p = 0.13$','$\sigma_p=0.19$','$\sigma_p=0.25$'},'Location','best'); 
ax.Legend.Interpreter = 'latex'; 
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';

% figure()
% yyaxis left
% plot(t,y(1,:),'LineWidth',1.5)
% yline(zmin,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% yline(cam.zMax,'--','LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% grid(gca,'minor')
% grid on
% ax = gca;
% ax.XLim = [0 100];
% ax.YLim = [0 2.2];
% ax.XLabel.FontSize = 15;
% ax.YLabel.FontSize = 15;
% ax.XLabel.Interpreter = 'latex';
% ax.XLabel.String = '$\gamma$';
% ax.YLabel.Interpreter = 'latex';
% ax.YLabel.String = '$I(z)$';
% 
% yyaxis right
% plot(t,y(2,:),'LineWidth',1.5)
% hold on
% yline(2,'--','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
% yline(3,'--','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
% grid(gca,'minor')
% grid on
% ax = gca;
% ax.XLim = [0 100];
% ax.YLim = [0 3.2];
% ax.XLabel.FontSize = 15;
% ax.YLabel.FontSize = 15;
% ax.YLabel.Interpreter = 'latex';
% ax.YLabel.String = '$u(k)$';


% legend({'$u(k)=2\Phi(k)-1$','$u(k)=1-e^{-{k \over b}}$'},'Location','southeast'); 
% ax.Legend.Interpreter = 'latex'; 
% ax.Legend.FontSize = 10;
% ax.Legend.Box = 'on';


