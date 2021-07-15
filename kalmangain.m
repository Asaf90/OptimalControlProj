function [ L ] = kalmangain( P,C,M )
%KALMANGAIN Summary of this function goes here
%   Detailed explanation goes here

L = P*C'*inv(C*P*C' + M);
end

