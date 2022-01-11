% Copyright (c) 2020, Nicolò Bargellesi, Luca Facin & Lorenzo Marchini
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree.
%

function [x_pred, P_pred] = kalman(A,H,Q,R,y,x,P)
%KALMAN Kalman Filter one step ahead predictor

    % Propagation
    if (y(1)==0 && y(2)==0)  
        H = 0.*H;
    end 

    % kalman gain matrix
    K = A*P*H'*(H*P*H' + R)^(-1);
    % prediction state
    x_pred = (A*x' + K*(y' - H*x'))';
    % covariance matrix of prediction errors
    P_pred = A*P*A' + Q - A*P*H'*(H*P*H' + R)^(-1)*H*P*A';
    

end

