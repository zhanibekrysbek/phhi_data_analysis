

%% Load the data

clc;clear;close all;
base_path = '../../data/preprocessed_v2_1';


[observations,tb] = load_data(base_path);
% tb = struct2table(observations);
% tb = convertvars(tb, {'motion_type','traj_type', 'obs_id'}, 'string');

%% 2D plot of all trajectories

figure(1);

for i = progress(1:numel(observations))
    obs = observations(i);
    if strcmp(obs.motion_type, 'parallel')
        h(1) = plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth', 0.5, 'Color', 'blue');
    elseif strcmp(obs.motion_type, 'serial')
        h(2) = plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth', 0.5, 'Color', 'magenta');
    else
        h(3) = plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth', 0.5, 'Color', 'green');
    end
    hold on;
end
title(sprintf('Motion types in 2D map%s', ''))
grid on; box on;
hold off;
axis equal;
% xlim([-0.1 2.9])
ylim([-1.4 1.2])
legend(h,{'parallel', 'serial', 'parallel->serial'})
xlabel('x-axis, m');
ylabel('y-axis, m');

rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',3)
text(-0.05,-0.6, 'A', 'FontSize',14)
rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',3)
text( 2.65,-0.6, 'B', 'FontSize',14)
rectangle('Position',[1.25 -0.2 .4 .4], 'EdgeColor', 'black', 'LineWidth',3)
text( 1.35,-0.05, 'Obs', 'FontSize',12)

%% Individual Trajectory
% close(h)
h = figure(11);
obs = observations(9);

subplot(2,2,1)
plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth',2);
grid on; axis equal; box on;
xlabel('x-axis, m');
ylabel('y-axis, m');
ylim([-1.4 1.2]);
subtitle('2D Trajectory')

subplot(2,2,3)
plot(obs.pose123.position(:,1), obs.pose123.linvel(:,1), 'LineWidth',2);
hold on;
plot(obs.pose123.position(:,1), obs.pose123.linvel(:,2), 'LineWidth',2);
plot(obs.pose123.position(:,1), vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);
hold off;
grid on; box on;
xlabel('x-axis, m');
ylabel('v_y, m/s');
ylim([-1.4 1.2])
legend('v_x', 'v_y', 'v_{xy}')
subtitle('Velocity (y-axis) vs horizontal Distance')

subplot(2,2,2)
plot(obs.pose123.time_steps, obs.pose123.linvel(:,1), 'LineWidth',2); hold on;
plot(obs.pose123.time_steps, obs.pose123.linvel(:,2), 'LineWidth',2);
plot(obs.pose123.time_steps, vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);hold off;
legend('v_x', 'v_y', 'v_{xy}')
grid on; box on;
xlabel('time, sec');
ylabel('v_x, m/s');
% ylim([min(min(obs.pose123.linvel(:,1:2))) max(max(obs.pose123.linvel(:,1:2)))])
subtitle('Velocity in x-axis vs time')


subplot(2,2,4)
plot(obs.pose123.position(:,2), obs.pose123.linvel(:,1), 'LineWidth',2);
hold on;
plot(obs.pose123.position(:,2), obs.pose123.linvel(:,2), 'LineWidth',2);
plot(obs.pose123.position(:,2), vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);
hold off
grid on; box on;
xlabel('time, sec');
ylabel('v_y, m/s');
ylim([-0.8 0.8])
subtitle('Velocity in y-axis vs time')
xlim([-0.8 0.8])

sgtitle('Single Example')


%% Parallel type movements

figure(2);

subplot(1,2,1)
hold on;
for i = progress(1:numel(observations),'Title', 'Position')
    obs = observations(i);
    if strcmp(obs.motion_type, 'parallel')
        plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth',1, 'Color', 'blue')
    end
end
rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',3)
text(-0.05,-0.6, 'A', 'FontSize',14)
rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',3)
text( 2.65,-0.6, 'B', 'FontSize',14)

rectangle('Position',[1.25 -0.2 .4 .4], 'EdgeColor', 'black', 'LineWidth',3)
text( 1.35,-0.05, 'Obs', 'FontSize',12)

title(sprintf('%s type movements', 'Parallel'))
grid on; hold off; axis equal; box on;
ylim([-1.4 1.2]);
xlim([-.2 2.9]);
xlabel('x-axis, m');
ylabel('y-axis, m');


% Serial type movements

subplot(1,2,2)
hold on;
for i = progress(1:numel(observations))
    obs = observations(i);
    if strcmp(obs.motion_type, 'serial')
        plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth',1, 'Color', 'blue')
    end
end
rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',3)
text(-0.05,-0.6, 'A', 'FontSize',14)
rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',3)
text( 2.65,-0.6, 'B', 'FontSize',14)

rectangle('Position',[1.25 -0.2 .4 .4], 'EdgeColor', 'black', 'LineWidth',3)
text( 1.35,-0.05, 'Obs', 'FontSize',12)

title(sprintf('%s type movements', 'Serial'))
grid on; hold off;axis equal;box on;
ylim([-1.4 1.2]);
xlim([-.2 2.9]);
xlabel('x-axis, m');
ylabel('y-axis, m');


%% Parallel and Serial type movements - Velocity
figure(21);
subplot(1,2,1)
hold on;
for i = progress(1:numel(observations), 'Title', 'Velocity')
    obs = observations(i);
    if strcmp(obs.traj_type, 'AB1')
        t = obs.pose123.time_steps;
        t = t/max(t);
        plot(t, obs.pose123.linvel(:,2), 'LineWidth',1, 'Color', 'blue')
    end
end
subtitle(sprintf('Trajectory type %s', 'AB1'))
grid on; hold off; box on;
xlabel('time, normalized');
ylabel('v_y, m/s');
ylim([-1,1])



subplot(1,2,2)
hold on;
for i = progress(1:numel(observations), 'Title', 'Velocity')
    obs = observations(i);
    if strcmp(obs.traj_type, 'AB2')
        t = obs.pose123.time_steps;
        t = t/max(t);
        plot(t, obs.pose123.linvel(:,2), 'LineWidth',1, 'Color', 'blue')
    end
end
subtitle(sprintf('Trajectory type %s', 'AB2'))
grid on; hold off; box on;
xlabel('time, normalized');
% ylabel('Velocity-axis, m/s');

sgtitle('Velocity along y-axis')


%% Orientation vs Motion Type


figure(33);
subplot(1,2,1)
hold on;
for i = progress(1:numel(observations), 'Title', 'Velocity')
    obs = observations(i);
    if strcmp(obs.motion_type, 'serial')
        orient = obs.pose123.orientation(:,4);
        orient = abs(orient - orient(1));
        t = obs.pose123.time_steps;
        t = t/t(end);
        plot(t, orient, 'LineWidth',1, 'Color', 'blue')
    end
end
subtitle(sprintf('Motion type %s', 'serial'))
grid on; hold off; box on;
xlabel('time, normalized');
ylabel('angle, rad');
ylim([-2.4,5])



subplot(1,2,2)
hold on;
for i = progress(1:numel(observations), 'Title', 'Velocity')
    obs = observations(i);
    if strcmp(obs.motion_type, 'parallel')
        orient = obs.pose123.orientation(:,4);
        orient = orient - orient(1);
        t = obs.pose123.time_steps;
        t = t/t(end);
        plot(t, orient, 'LineWidth',1, 'Color', 'blue')
    end
end
subtitle(sprintf('Motion type %s', 'parallel'))
grid on; hold off; box on;
xlabel('time, normalized');
sgtitle('Motion type vs Orientation')
ylim([-2.4,5])


%% Overall Summary

mtype_count = [sum(tb.motion_type=='parallel'), sum(tb.motion_type=='serial')];
mlabels = {'parallel '; 'serial '};
ttype_count = [sum(tb.traj_type=='AB1'), sum(tb.traj_type=='AB2')];

mtype_ab1 = [height(tb(tb.traj_type=='AB1'&tb.motion_type=='parallel',:)),...
            height(tb(tb.traj_type=='AB1'&tb.motion_type=='serial',:))];
mtype_ab2 = [height(tb(tb.traj_type=='AB2'&tb.motion_type=='parallel',:)),...
            height(tb(tb.traj_type=='AB2'&tb.motion_type=='serial',:))];
        
tlabels = {'AB1 '; 'AB2 '};

figure(100)
subplot(2,2,1)
p=pie(mtype_count);
title('Parallel vs Serial')
legend(mlabels)

subplot(2,2,2)
p=pie(ttype_count);

title('AB1 vs AB2')
legend(tlabels)

subplot(2,2,3)
p=pie(mtype_ab1);
title('Parallel vs Serial for "AB1"')
legend(mlabels, 'Location', 'south east')

subplot(2,2,4)
p=pie(mtype_ab2);

title('Parallel vs Serial for "AB2"')
% legend(mlabels)
% subtitle('AB1 vs AB2')

sgtitle('Distribution')


%% Duration summary


BW = 0.5;

figure(71);
histogram(tb.duration, 'BinWidth', BW); grid on;
subtitle('Overall Duration of observations')
xlabel('time, sec')
ylabel('counts')

figure(72);
subplot(1,2,1)
histogram(tb(tb.motion_type=='parallel',:).duration, 'BinWidth', BW); grid on;
xlabel('time, sec')
ylabel('counts')
subtitle('Parallel Movements')

subplot(1,2,2)
histogram(tb(tb.motion_type=='serial',:).duration, 'BinWidth', BW); grid on;
xlabel('time, sec')
% ylabel('counts')
subtitle('Serial Movements')


figure(73);
subplot(1,2,1)
histogram(tb(tb.traj_type=='AB1',:).duration, 'BinWidth', BW); grid on;
xlabel('time, sec')
ylabel('counts')
subtitle('AB1 trajectory')

subplot(1,2,2)
histogram(tb(tb.traj_type=='AB2',:).duration, 'BinWidth', BW); grid on;
xlabel('time, sec')
% ylabel('counts')
subtitle('AB2 trajectory')


