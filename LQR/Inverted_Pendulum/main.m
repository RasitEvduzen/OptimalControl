clc,clear all,close all;
% Inverted Pendulum LQR based Control
% 21-Feb-2020
% Rasit EVDUZEN

% System Parameter
m = 5;   % pendulum mass
M = 5;   % cart mass
L = 2;   % pendulum length
g = -9.81; % gravitational force
d = 10;   % cart damping
b = 1;   % pendulum up (b=1)

% Linear State Space Model
A = [0  1          0               0;
    0 -d/M        b*m*g/M         0;
    0  0          0               1;
    0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; b*1/(M*L)];

%  Design LQR controller
Q = 1e2*eye(length(A));
R = 1e-3;
K = lqr(A,B,Q,R);

Control = "Off"; % Control (On - Off)

% Simulate closed-loop system
tspan = 0:2e-3:16;
xini = [-6; 0; pi+1e-1; 0];  % initial condition
Ref = [6; 0; pi; 0];       % reference position

if Control == "On"
    u  = @(x)-K*(x - Ref);     % control law
else
    u = @(x) 0;      % Non Force Solution
end
[t,state] = ode45(@(t,x) PendStateSpace(x,m,M,L,g,d,u(x)),tspan,xini);  % Runge Kutta4 for Forward Dynamics Simulation


%% PLOT Simulation
figure('units','normalized','outerposition',[0 0 1 1],'color','w')
for k=1:1e2:length(t)
    x = state(k,1);
    th = state(k,3);
    % dimensions
    W  = 1.5*sqrt(M/5);  % cart width
    H  = .5*sqrt(M/5);   % cart height
    wr = .5;             % wheel radius
    mr = .3*sqrt(m);     % mass radius
    % positions
    y = wr/2+H/2; % cart vertical position
    pendx = x + L*sin(th);
    pendy = y - L*cos(th);
    % Plot Pendulum
    clf
    subplot(2,2,[1,2])
    yline(0,'k--','LineWidth',2), hold on
    rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[0.4940 0.1840 0.5560],'LineWidth',1.5); % Draw cart
    rectangle('Position',[x-.9*W/2,0,wr,wr],'Curvature',1,'FaceColor',[1 1 0],'LineWidth',1.5);    % Draw wheel
    rectangle('Position',[x+.9*W/2-wr,0,wr,wr],'Curvature',1,'FaceColor',[1 1 0],'LineWidth',1.5); % Draw wheel
    plot([x pendx],[y pendy],'k','LineWidth',2); % Draw pendulum
    rectangle('Position',[pendx-mr/2,pendy-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
    axis equal,axis([-12 12 -3 3]),grid on
    title('Inverted Pendulum LQR Based Control')
    xline(xini(1),'b'),xline(Ref(1),'b')

    subplot(2,2,[3,4])
    plot(t,state,'LineWidth',2); hold on
    xline(t(k),'r')
    legend('x','v','\theta','\omega','time','Location','SouthEast');
    xlabel('Time'),ylabel('State'),grid on

    drawnow
end

% Phase Space Plot
figure('units','normalized','outerposition',[0 0 1 1],'color','w')
subplot(121)
plot(state(:,1),state(:,2),"b",LineWidth=3),title("Pendulum Phase Space"),grid
subplot(122)
plot(state(:,3),state(:,4),"b",LineWidth=3),title("Pendulum Phase Space"),grid

function dx = PendStateSpace(x,m,M,L,g,d,u)
D = m*L*L*(M+m*(1-cos(x(3))^2));
dx(1,1) = x(2);
dx(2,1) = (1/D)*(-m^2*L^2*g*cos(x(3))*sin(x(3)) + m*L^2*(m*L*x(4)^2*sin(x(3)) - d*x(2))) + m*L*L*(1/D)*u;
dx(3,1) = x(4);
dx(4,1) = (1/D)*((m+M)*m*g*L*sin(x(3)) - m*L*cos(x(3))*(m*L*x(4)^2*sin(x(3)) - d*x(2))) - m*L*cos(x(3))*(1/D)*u;
end