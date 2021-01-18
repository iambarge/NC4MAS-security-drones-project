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

