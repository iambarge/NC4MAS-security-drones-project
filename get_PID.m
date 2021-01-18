function [kp, ki, kd] = get_PID(sysP, wgc, phim, alpha)
%GET_PID  PID controller design with Bode's method
%
%   [KP, KI,KD] = GET_PID(SYSP, WGC, PHIM, ALPHA) designs a PID controller 
%   with Bode's method for the plant SYSP, using WGC [rad/s] as gain 
%   crossover frequency and PHIM [deg] as phase margin. ALPHA is the ratio 
%   between the integral time TI=KP/KI and the derivative time TD=KD/KP, 
%   i.e. ALPHA = TI/TD.
%
%   The controller tf is equal to (in the continuous-time case):
%       C(s) = (KP + KI/s + KD*s) = KP*(1 + 1/(s*TI) + s*TD)
%
%   The design is performed either in continuous or discrete time,
%   depending on the plant tf SYSP. The integrator tf in the discrete time
%   case is considered as Ts*z/(z-1) (Backward Euler). 

%%
% Riccardo Antonello (riccardo.antonello@unipd.it)
%
% January 28, 2018
%
% Dept. of Information Engineering, University of Padova

