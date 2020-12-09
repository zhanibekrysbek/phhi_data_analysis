function [] = plot_pose(obs, opt)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if opt==1
    
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
    
    
elseif opt==2
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
end
end

