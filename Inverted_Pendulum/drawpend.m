function drawpend(state,m,M,L)
x = state(1);
th = state(3);

% dimensions
W  = 1*sqrt(M/5);  % cart width
H  = .5*sqrt(M/5); % cart height
wr = .2;           % wheel radius
mr = .3*sqrt(m);   % mass radius

% positions
y = wr/2+H/2; % cart vertical position
pendx = x + L*sin(th);
pendy = y - L*cos(th);


plot([-10 10],[0 0],'k--','LineWidth',2), hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart
rectangle('Position',[x-.9*W/2,0,wr,wr],'Curvature',1,'FaceColor',[1 1 0],'LineWidth',1.5); % Draw wheel
rectangle('Position',[x+.9*W/2-wr,0,wr,wr],'Curvature',1,'FaceColor',[1 1 0],'LineWidth',1.5); % Draw wheel
plot([x pendx],[y pendy],'k','LineWidth',2); % Draw pendulum
rectangle('Position',[pendx-mr/2,pendy-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
axis([-5 5 -1 3]);, axis equal,grid on
title('Inverted Pendulum LQR Based Control')
drawnow,hold off
