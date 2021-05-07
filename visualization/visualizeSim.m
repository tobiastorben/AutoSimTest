%© Norwegian University of Science and Technology (NTNU),
%Department of Marine Technology.
%The software is developed under the ORCAS Project.
%Author: Tobias Rye Torben.

function visualizeSim(datapoint, rho_coll, rho_colreg, rho_mission, rtf, north_lim, east_lim, file)
L = 20;B=6;
h = figure();
h1 = subplot(3,2,[1 3 5]);
colors = {'#0072BD' '#D95319' '#EDB120' '#7E2F8E' '#77AC30' '#4DBEEE' '#A2142F'};
time = datapoint.t;
dt = time(2)-time(1);
writerObj = VideoWriter(file);
writerObj.FrameRate = rtf/dt;
open(writerObj);
i_t = 1;
xlabel('East [m]');
ylabel('North [m]');
axis equal;
axis manual;
grid on;
xlim(east_lim);ylim(north_lim);

traj_handle(1) = animatedline('LineWidth', 1, 'Color', colors{1});
poly = get_polygon(datapoint.eta(:,i_t), L, B);
poly_handle(1) = patch('XData',poly(2,:),'YData',poly(1,:),'FaceColor','b');
names{1} = 'Ownship';
for i = 1:length(datapoint.obs_eta)
    traj_handle(i+1) = animatedline('LineWidth', 1, 'Color', colors{i+1});
    poly = get_polygon(datapoint.obs_eta{i}(:,i_t), L, B);
    poly_handle(i+1) = patch('XData',poly(2,:),'YData',poly(1,:),'FaceColor','k');
    names{i+1} = strcat('Obstacle ',num2str(i));
end
legend(traj_handle, names);
subplot(3,2,2)
coll_handle = animatedline('LineWidth', 1);
title('Safety Robustness score')
xlim([time(1) time(end)])
lim = 1.3;
ylim([-lim  lim])
grid on;

subplot(3,2,4)
mission_handle = animatedline('LineWidth', 1);
title('Mission Robustness score')
xlim([time(1) time(end)])
ylim([-lim  lim])
grid on;

subplot(3,2,6)
xlim([time(1) time(end)])
ylim([-lim  lim])
colreg_handle = animatedline('LineWidth', 1);
title('COLREG Robustness score')
xlabel('Time [s]')
grid on;
subplot(3,2,[1 3 5]);
for t = time(1:end-1)
    tic;
    dt = time(i_t+1) - time(i_t);
    addpoints(traj_handle(1), datapoint.eta(2,i_t), datapoint.eta(1,i_t));
    poly = get_polygon(datapoint.eta(:,i_t), L, B);
    poly_handle(1).XData = poly(2,:);
    poly_handle(1).YData = poly(1,:);
    for i = 1:length(datapoint.obs_eta)
        addpoints(traj_handle(i+1), datapoint.obs_eta{i}(2,i_t), datapoint.obs_eta{i}(1,i_t));
        poly = get_polygon(datapoint.obs_eta{i}(:,i_t), L, B);
        poly_handle(i+1).XData = poly(2,:);
        poly_handle(i+1).YData = poly(1,:);
    end
    addpoints(coll_handle, t, rho_coll(i_t));
    addpoints(colreg_handle, t, rho_colreg(i_t));
    addpoints(mission_handle, t, rho_mission(i_t));
    drawnow;
    writeVideo(writerObj, getframe(h));
    i_t = i_t + 1;
    pause_time = max(dt/rtf - toc, 0.0);
    pause(pause_time)
    
end
close(writerObj);
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