clc
close all
clear variables

%define timeline
dt = 0.1;
tf = 10;
t = 0:dt:tf-dt;
N = length(t);
%define parameter values:
V = 3000 * 3.28; %convert to m/sec for consistency
tau = 2;
R1 = 15e-6;
R2 = 1.67e-3;
W_cont = 100;
b = 1.52e-2;
P22 = 16;
P33 = 400;


Q_cont = zeros(3);
R_cont = b/2;
Sf = [0.5 0 0; 0 0 0; 0 0 0];
P_0 = diag([0 P22 P33]);

F_cont = [0 1 0; 0 0 -1; 0 0 1];
B_cont = [0; 1; 0];

create_H = 1./(V*(tf-t))';              %create vector of values for first element of H
H_cont = zeros(length(create_H), 3);    
H_cont(1:length(create_H)) = create_H;
M_cont = R1 + R2/((tf-t').^2);
G = [0; 0; 1];

F = exp(F_cont*dt);
B = B_cont*dt;
M = M_cont/dt;
H = H_cont;
Q = Q_cont;
R = R_cont;
W = G*W_cont*G'*dt;

a_T = zeros(1,N);         
y = zeros(1,N);
v = zeros(1,N);

% P = zeros(3,3,N);
% P(:,:,1) = P_0;
% x_hat = zeros(3,N+1);
% J = zeros(1,N);


x = [y; v; a_T];            % state vecvor
z = zeros(size(x));      % create measurement vector
% u = zeros(1,N);     % plant vector


[ u,x_hat, J,P] = lqgcontrol( x,z,F,B,H,M,Q,R,Sf,P_0,W,N );

figure;
plot(t,u)
title('Estimated Control Signal')
xlabel('Time[sec]');
ylabel('Control - Signal u_k')

figure;
plot(t,J)
title('Estimated Cost')
xlabel('Time[sec]');
ylabel('Cost - J_k')

figure;
plot(t,x_hat(3,1:end-1))
hold on
plot(t,x_hat(2,1:end-1))
plot(t,x_hat(1,1:end-1))
title('Estimated State Vector')
xlabel('Time[sec]');
ylabel('State - x_k')
legend('a_t','v','y','Location','southwest')


% mu - 1;
% sigma = 1;
% a_T = bm(mu,sigma);