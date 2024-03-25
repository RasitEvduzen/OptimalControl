%% Pole Placement Design via Ackerman VS LQR
% Written By: Rasit Evduzen
% 20-May-2020
clc,clear all,close all;
% System Matrix
A = [0 1 0; 0 0 1; -1 -5 -6];
B = [0 0 1]';
J = [-2+4*j -2-4*j -10];  % Close Loop Pole Location
K = acker(A,B,J);         % Full State Feedback Gain

Q = eye(3);
R = 1;
Kopt = lqr(A,B,Q,R);     % Full State Feedback Optimal Gain

sys1 = ss((A-B*K),eye(3),eye(3),eye(3));     % Ackerman State Space Solution
sys2 = ss((A-B*Kopt),eye(3),eye(3),eye(3));  % Optimal State Space Solution

t = 0:5e-2:15;   % Simulation Time
% Ackerman
xa = initial(sys1,[1 2 3]',t);   % System initial State
% LQR
xb = initial(sys2,[1 2 3]',t);   % System initial State


%% Plot
figure('units','normalized','outerposition',[0 0 1 1],'color','w')

subplot(3,2,1)
plot(t,xa(:,1),'k','LineWidth',2),grid,hold on
plot(t,xb(:,1),'r','LineWidth',2)
ylabel('State X1'),xlabel('Time'),legend('Ackerman','LQR')

subplot(3,2,3)
plot(t,xa(:,2),'k','LineWidth',2),grid,hold on
plot(t,xb(:,2),'r','LineWidth',2)
ylabel('State X2'),xlabel('Time'),legend('Ackerman','LQR')


subplot(3,2,5)
plot(t,xa(:,3),'k','LineWidth',2),grid,hold on
plot(t,xb(:,3),'r','LineWidth',2)
ylabel('State X3'),xlabel('Time'),legend('Ackerman','LQR')


subplot(3,2,[2,4,6])
plot3(xa(:,1),xa(:,2),xa(:,3),'k','LineWidth',2),grid,hold on
plot3(xb(:,1),xb(:,2),xb(:,3),'r','LineWidth',2)
title('System Phase Space'),legend('Ackerman','LQR')
xlabel('State X1'),ylabel('State X2'),zlabel('State X3')
