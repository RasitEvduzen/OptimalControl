clc,clear all,close all;
% Inverted Pendulum LQR based Control
% 21-Feb-2020
% Rasit EVDÜZEN

% System Parameter
m = 1;   % pendulum mass
M = 5;   % cart mass
L = 2;   % pendulum length
g = -10; % gravitational force
d = 1;   % cart damping
b = 1;   % pendulum up (b=1) there are 2 fixed here, [0 - down] [pi - up]

% Linear State Space Model 
A = [0  1          0               0;
     0 -d/M        b*m*g/M         0;
     0  0          0               1;
     0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; b*1/(M*L)];

%%  Design LQR controller
Q = eye(length(A));
R = 0.0001;
K = lqr(A,B,Q,R);


%% Simulate closed-loop system
tspan = 0:.001:10;
x0 = [-2; 0; pi; 0];  % initial condition 
wr = [2; 0; pi; 0];       % reference position
u  = @(x)-K*(x - wr);     % control law
[t,x] = ode45(@(t,x)pendcart(x,m,M,L,g,d,u(x)),tspan,x0);  % Runge Kutta4 Based Simulation


%% PLOT 
figure,set(gcf,'Position',[100 100 1000 800]) 
subplot(2,2,[1,2])
for k=1:100:length(t)
    drawpend(x(k,:),m,M,L);
end
hold on
subplot(2,2,[3,4])
plot(t,x,'LineWidth',2); hold on
l1 = legend('x','v','\theta','\omega');
set(l1,'Location','SouthEast')
xlabel('Time'),ylabel('State'),grid on
