function [ K ] = feedbackgain( B,S,R,F )
%FEEDBACKGAIN Summary of this function goes here
%   Detailed explanation goes here

K = inv(B'*S*B + R)*B'*S*F;
end

