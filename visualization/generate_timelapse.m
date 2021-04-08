function generate_timelapse(datapoint, rho_coll, rho_colreg, rho_mission)
L = 20;B=6;
h = figure();
t_snap = [40, 50, 64];
colors = {'#0072BD' '#D95319' '#EDB120' '#7E2F8E' '#77AC30' '#4DBEEE' '#A2142F'};
time = datapoint.t;
i_t = 1;
[~, i_snap(1)] = min(abs(time-t_snap(1)));
[~, i_snap(2)] = min(abs(time-t_snap(2)));
[~, i_snap(3)] = min(abs(time-t_snap(3)));

h1 = subplot(3,4,[1 5 9]);
plot(datapoint.eta(2,1:i_snap(1)), datapoint.eta(1,1:i_snap(1)));
hold on;
plot(datapoint.obs_eta{1}(2,1:i_snap(1)), datapoint.obs_eta{1}(1,1:i_snap(1)));
ylabel('North [m]');
title(sprintf('t = %ds', t_snap(1)));
axis equal;
axis manual;
set(gca,'xticklabel',[])
grid on;
xlim([-150 150]);ylim([0 1000]);
poly = get_polygon(datapoint.eta(:,i_snap(1)), L, B);
patch('XData',poly(2,:),'YData',poly(1,:),'FaceColor','b');
poly = get_polygon(datapoint.obs_eta{1}(:,i_snap(1)), L, B);
patch('XData',poly(2,:),'YData',poly(1,:),'FaceColor','k');


h2 = subplot(3,4,[2 6 10]);
plot(datapoint.eta(2,1:i_snap(2)), datapoint.eta(1,1:i_snap(2)));
hold on;
plot(datapoint.obs_eta{1}(2,1:i_snap(2)), datapoint.obs_eta{1}(1,1:i_snap(2)));
xlabel('East [m]');
title(sprintf('t = %ds', t_snap(2)));
axis equal;
axis manual;
set(gca,'yticklabel',[])
grid on;
xlim([-150 150]);ylim([0 1000]);
poly = get_polygon(datapoint.eta(:,i_snap(2)), L, B);
patch('XData',poly(2,:),'YData',poly(1,:),'FaceColor','b');
poly = get_polygon(datapoint.obs_eta{1}(:,i_snap(2)), L, B);
patch('XData',poly(2,:),'YData',poly(1,:),'FaceColor','k');



h3 = subplot(3,4,[3 7 11]);
plot(datapoint.eta(2,1:i_snap(3)), datapoint.eta(1,1:i_snap(3)));
hold on;
plot(datapoint.obs_eta{1}(2,1:i_snap(3)), datapoint.obs_eta{1}(1,1:i_snap(3)));
title(sprintf('t = %ds', t_snap(3)));
axis equal;
axis manual;
set(gca,'xticklabel',[], 'yticklabel',[])
grid on;
xlim([-150 150]);ylim([0 1000]);
poly = get_polygon(datapoint.eta(:,i_snap(3)), L, B);
patch('XData',poly(2,:),'YData',poly(1,:),'FaceColor','b');
poly = get_polygon(datapoint.obs_eta{1}(:,i_snap(3)), L, B);
patch('XData',poly(2,:),'YData',poly(1,:),'FaceColor','k');

subplot(3,4,4)
coll_handle = plot(time, rho_coll, 'k', 'LineWidth', 1);
title('Safety Robustness')
xlim([time(1) time(i_snap(end))])
lim = 1.3;
ylim([-lim  lim])
grid on;

subplot(3,4,8)
mission_handle = plot(time, rho_mission, 'k', 'LineWidth', 1);
title('Mission Robustness')
xlim([time(1) time(i_snap(end))])
ylim([-lim  lim])
grid on;

subplot(3,4,12)
colreg_handle = plot(time, rho_colreg, 'k', 'LineWidth', 1);
xlim([time(1) time(i_snap(end))])
ylim([-lim  lim])
title('COLREG Robustness')
xlabel('Time [s]')
grid on;
end

function poly = get_polygon(eta,L,B)
Xi = eta(3);
p = eta(1:2);

%Local
poly = [-L/2 L/6 L/2 L/6 -L/2;
    -B/2 -B/2 0 B/2 B/2];

%Rotated
R = [cos(Xi) -sin(Xi);
    sin(Xi) cos(Xi)];
poly = R*poly;

%Translated
poly = poly + p;
end