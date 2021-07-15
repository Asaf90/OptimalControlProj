function [ P1 ] = probMat( F,P,C,M,W )
%PROBMAT Summary of this function goes here
%   Detailed explanation goes here

P1 = F*(P - P*C'*inv(C*P*C' + M)*C*P)*F' + W;
end

