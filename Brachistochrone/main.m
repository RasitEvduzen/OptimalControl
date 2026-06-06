% Brachistochrone Problem  Animation
% 3 curves: Cycloid (optimal), Straight line, Deep sine
% Written By: Rasit Evduzen
% Date: 06-Jun-2026
clear; clc; close all;
%%
g  = 9.81;
x1 = 1.0;
y1 = 0.8;
N  = 1000;

% CYCLOID
sol   = fsolve(@(v) [v(1)*(v(2)-sin(v(2))) - x1; ...
    v(1)*(1  -cos(v(2))) - y1], ...
    [0.5, pi], optimset('Display','off'));
r_opt = sol(1);
t_end = sol(2);
t_c   = linspace(0, t_end, N);
x_cyc = r_opt*(t_c - sin(t_c));
y_cyc = r_opt*(1   - cos(t_c));

% STRAIGHT LINE
x_line = linspace(0, x1, N);
y_line = linspace(0, y1, N);

% DEEP SINE  overshoots optimal depth, slowest
x_sha  = linspace(0, x1, N);
y_sha  = y1 * sin(pi/2 * x_sha/x1);

T_cyc  = compute_time(x_cyc,  y_cyc,  g);
T_line = compute_time(x_line, y_line, g);
T_sha  = compute_time(x_sha,  y_sha,  g);
T_max  = max([T_cyc, T_line, T_sha]) * 1.05;

fps = 30;
dt  = 1/fps;

c_cyc  = [0.15 0.40 0.85];
c_line = [0.85 0.20 0.20];
c_sha  = [0.80 0.50 0.05];

curves = {x_cyc,  y_cyc,  T_cyc,  c_cyc,  'Cycloid (optimal)'; ...
    x_line, y_line, T_line, c_line, 'Straight line'; ...
    x_sha,  y_sha,  T_sha,  c_sha,  'Deep sine curve'};

t_phys_all = cell(3,1);
for k = 1:3
    xk    = curves{k,1}; yk = curves{k,2};
    ds    = sqrt(diff(xk).^2 + diff(yk).^2);
    y_mid = (yk(1:end-1) + yk(2:end)) / 2;
    v_mid = sqrt(2*g*max(y_mid, 1e-9));
    t_phys_all{k} = [0, cumsum(ds./v_mid)];
end

%% Simulation
figure('Position', [0 0 1920 1080], 'Color', 'w');
t_now = 0;
while t_now <= T_max
    clf; set(gcf, 'Color', 'w');
    draw_frame(curves, t_phys_all, t_now, x1, y1);
    drawnow;
    t_now = t_now + dt;
end

%% Utility Function
function draw_frame(curves, t_phys_all, t_now, x1, y1)

ax = axes('Color', [0.96 0.96 0.96]);
hold on; grid on; box on;
set(ax, 'YDir', 'reverse', 'FontSize', 11);

for k = 1:3
    plot(curves{k,1}, curves{k,2}, '-', ...
        'Color', curves{k,4}, 'LineWidth', 2);
end

plot(0,  0,  'ko', 'MarkerSize', 10, 'MarkerFaceColor', [0.3 0.3 0.3]);
plot(x1, y1, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', [0.3 0.3 0.3]);

for k = 1:3
    xk     = curves{k,1};
    yk     = curves{k,2};
    Tk     = curves{k,3};
    ck     = curves{k,4};
    t_phys = t_phys_all{k};

    if t_now >= Tk
        idx = length(xk);
    else
        idx = find(t_phys >= t_now, 1, 'first');
        if isempty(idx), idx = length(xk); end
    end
    idx = max(idx, 1);

    plot(xk(1:idx), yk(1:idx), '-', 'Color', ck, 'LineWidth', 4);
    plot(xk(idx), yk(idx), 'o', 'MarkerSize', 14, ...
        'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineWidth', 2);
end

text(0.02, -0.13, sprintf('t = %.3f s', t_now), ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);

for k = 1:3
    ly = 0.03 + (k-1)*0.08;
    plot([0.75 0.82], [ly ly], '-', 'Color', curves{k,4}, 'LineWidth', 4);
    plot(0.785, ly, 'o', 'MarkerSize', 10, ...
        'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
    text(0.84, ly, sprintf('%s  (%.3f s)', curves{k,5}, curves{k,3}), ...
        'FontSize', 11, 'Color', [0.15 0.15 0.15], 'FontWeight', 'bold');
end

title('Brachistochrone Bead Race', 'FontSize', 15, 'FontWeight', 'bold');
xlim([-0.05 x1*1.32]);
ylim([-0.18  y1*1.10]);
ylabel('y (m, downward +)', 'FontSize', 13);
xlabel('x (m)', 'FontSize', 13);
end


function T = compute_time(x, y, g)
ds    = sqrt(diff(x).^2 + diff(y).^2);
y_mid = (y(1:end-1) + y(2:end)) / 2;
v_mid = sqrt(2*g*max(y_mid, 1e-9));
T     = sum(ds./v_mid);
end