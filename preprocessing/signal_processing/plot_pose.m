function [] = plot_pose(obs)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

subplot(211)
plot(obs.pose123.time_steps, obs.pose123.position,'.')
% plot(obs.pose123.position,'.')
grid on;
subtitle('Position')
ylabel('m')
legend('x','y','z')

subplot(212)
% plot(obs.pose123.orientation,'.')
plot(obs.pose123.time_steps, obs.pose123.orientation,'.')

grid on;
subtitle('Orientation')
xlabel('time, s')
ylabel('axis-angle')
legend('x','y','z','angle')

sgtitle(sprintf('%s %s %s Position', obs.obs_id, obs.traj_type, obs.motion_type ), ...
        'Interpreter','none');
end

