%% Mean Idleness
X = randi([2,5],1,5);
I=[];
l=0;
figure()
hold on
axis equal
for x = X(1:end-1)
    I = [I 0:x];
   area(l:l+x,0:x,'FaceColor',[0, 0.4470, 0.7410],'FaceAlpha',0.25)
   scatter(l+x,x,30,[0.6350, 0.0780, 0.1840],'filled')
   scatter(l+x,0,30,[0, 0, 0],'filled')
   l=l+x;
end
I = [I 0:X(end)/2];
area(l:l+X(end)/2,0:X(end)/2,'FaceColor',[0, 0.4470, 0.7410],'FaceAlpha',0.25)
plot(l:l+X(end),0:X(end),'Color',[0, 0.4470, 0.7410])

l=l+X(end);

yline(mean(X),'--','Color',[0.6350, 0.0780, 0.1840],'LineWidth',1)
yline(mean(I),'--','Color',[0, 0.4470, 0.7410],'LineWidth',1)

ylim([0,max(X)+1])
xlim([0,l+1])

%% Compared Time
TcO = [31 13 11 5 3 3 1];
TcS = [31.113 13.654 10.291 5.564 3.218 3.148 1.674];

TlO = [17 9 5 3 1];
TlS = [16.998 9.878 5.247 2.997 1.812];

ToO = [13 7 5 1];
ToS = [12.998 7.017 4.684 1.000];

T = mean([TcS-TcO, TlS-TlO, ToS-ToO]);

figure()
plot(abs(TcO-TcS))
hold on
plot(abs(TlO-TlS))
plot(abs(ToO-ToS))

figure()
plot((TcS./TcO)-1)
hold on
plot((TlS./TlO)-1)
plot((ToS./ToO)-1)

%% Compared idleness
IcO = [11.478 4.295 2.395 1.100 0.583 0.361 0.250];
IcS = [11.517 4.486 2.562 1.272 0.688 0.422 0.264];

IlO = [6.224 2.169 0.875 0.417 0.188];
IlS = [6.228 2.551 0.976 0.515 0.248];

IoO = [4.753 1.687 0.657 0.143];
IoS = [4.755 1.870 0.775 0.206];

I = mean([IcS(1:end-1)./IcO(1:end-1), IlS(1:end-1)./IlO(1:end-1), IoS(1:end-1)./IoO(1:end-1)])-1;

figure()
plot(abs(IcO-IcS))
hold on
plot(abs(IlO-IlS))
plot(abs(IoO-IoS))

figure()
plot((IcS./IcO)-1)
hold on
plot((IlS./IlO)-1)
plot((IoS./IoO)-1)

%% Steps REAL
I = [38.113 18.718 11.905 8.289 6.115 4.682 3.664 2.908 2.337 1.888];
T = [135.849 64.100 44.468 32.683 25.651 21.070 17.783 15.170 13.067 11.326];
D = [28.908 13.687 8.356 5.999 4.073 3.087 2.470 1.960 1.263 0.970];

Dd = mean(I./D)-1;

Id = I(1:end-1)./I(2:end)-1;
Td = T(1:end-1)./T(2:end)-1;

figure()


%yyaxis left
plot(Id,'linewidth',1)
%ylabel('$\Delta\overline{\mathcal{I}_{\mathcal{G}}}$ [\%]','interpreter','latex','FontSize', 25)
hold on
%yyaxis right
plot(Td,'linewidth',1)
ylabel('$\Delta_{\%}$','interpreter','latex','FontSize', 25)
xlabel('$\Delta n_p$','interpreter','latex','FontSize', 20)
grid on
ax = gca;
ax.XLabel.FontSize = 18;
ax.YLabel.FontSize = 18;
legend('$\Delta \overline{\mathcal{I}_c}_{\%}$','$\Delta \overline{T_c}_{\%}$')
ax.Legend.Interpreter = 'latex'; 
ax.Legend.FontSize = 10;
ax.Legend.Box = 'on';
ax.Legend.Location = 'southeast';


