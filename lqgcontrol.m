function [ u,x_hat, J,P] = lqgcontrol( x,z,F,B,H,M,Q,R,S,P0,W,N )
%LQGCONTROL Summary of this function goes here
%   Detailed explanation goes here
if N ~= 1
    if N ~= length(x)
        S = F'*S*F - F'*S*B*inv(B'*S*B+R)*B'*S*F + Q;
    end
    [u,x_hat,J,P] = lqgcontrol(x,z,F,B,H,M,Q,R,S,P0,W,N-1);
    
    K = feedbackgain(B,S,R,F);      % calculate feedback gain matrix
    P = probMat(F,P,H(N,:),M(:,N),W);         % calculate propagation of probability matrix
    L = kalmangain(P,H(N,:),M(:,N));          % calculate Kalman gain matrix
    
    z(:,N) = H(N,:)*x_hat(:,N) + M(:,N);
    u(1,N) = -K*x_hat(:,N);
    x_hat(:,N+1) = F*x_hat(:,N) + B*u(1,N) + L'*(z(:,N) - H(N,:)*(F*x_hat(:,N) + B*u(1,N)));
    J(1,N) = x_hat(:,N)'*Q*x_hat(:,N) + u(1,N)'*R*u(1,N);
    
else
    K = feedbackgain(B,S,R,F);      % calculate feedback gain matrix
    P = probMat(F,P0,H(N,:),M(:,N),W);        % calculate propagation of probability matrix
    L = kalmangain(P,H(N,:),M(:,N));          % calculate Kalman gain matrix
    
    x_hat = zeros(3,length(x)+1);
    u = zeros(1,length(x));
    J = zeros(1,length(x));
    
    z(:,N) = H(N,:)*x(:,N) + M(:,N);
    u(1,N) = -K*x(:,N);
    x_hat(:,N+1) = F*x_hat(:,N) + B*u(1,N) + L'*(z(:,N) - H(N,:)*(F*x_hat(:,N) + B*u(1,N)));
    J(1,N) = x_hat(:,N)'*Q*x_hat(:,N) + u(1,N)'*R*u(1,N);
end
end

