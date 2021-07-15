function [ u,x_hat, J,P] = lqgcontrol( x,z,F,B,H,M,Q,R,S,P0,N )
%LQGCONTROL Summary of this function goes here
%   Detailed explanation goes here
if N ~= 1
    if N ~= length(x,3)
        S = F'*S*F - F'*S*B*inv(B'*S*B+R)*B'*S*A + Q;
    end
    [u,x_hat,J,P] = lqgcontrol(x,z,F,B,H,M,Q,R,S,P0,N-1);
    
    K = feedbackgain(B,S,R,F);
    P = probMat(F,P,C,M,W);
    L = kalmangain(P,C,M);
    
    u = -K*x;
    x_hat = A*x + B*u + L*(z - C*(A*x + B*u));
    J = x'*Q*x + u'*R*u;
    
else
    K = feedbackgain(B,S,R,F);
    P = probMat(F,P0,C,M,W);
    L = kalmangain(P,C,M);
    
    u = -K*x;
    x_hat = A*x + B*u + L*(z - C*(A*x + B*u));
    J = x'*Q*x + u'*R*u;
end
end

