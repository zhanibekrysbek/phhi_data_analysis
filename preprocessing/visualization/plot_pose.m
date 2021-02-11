function [] = plot_pose(obs, opt)
%plot_pose Summary of this function goes here
%   Detailed explanation goes here
switch opt
    % Plot Pose vs time
    case 1
        subplot(211)
        plot(obs.pose123.time_steps, obs.pose123.position)
        % plot(obs.pose123.position,'.')
        grid on;
        subtitle('Position')
        ylabel('m')
        xlabel('time, s')
        xts = xticks;
        xticks(xts(1):xts(end));


        subplot(212)
        % plot(obs.pose123.orientation,'.')
        plot(obs.pose123.time_steps, obs.pose123.orientation)
        xticks(xts(1):xts(end))

        grid on;
        subtitle('Orientation')
        xlabel('time, s')
        ylabel('axis-angle')
        hL = legend({'x', 'y', 'z', 'angle'},'Location','southwest','NumColumns',4);

        sgtitle(sprintf('%s %s %s Position', obs.obs_id, obs.traj_type, obs.motion_type ), ...
                'Interpreter','none');

        set(hL, 'Position',[0.2 0.03 0.01 0.01],'Units','normalized')

    
    case 2
        % Mix plots of 2D map, velocity and orientation vs space
        subplot(2,2,1)
        plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth',1.5);
        grid on; axis equal; box on;
        xlabel('x-axis, m');
        ylabel('y-axis, m');
        ylim([-1.4 1.2]);
        subtitle('2D Trajectory')
        xts = xticks;
        xl1 = xlim;
        xticks(xts(1):0.5:xts(end));
        % Plot tables and obstacle
        rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',1.)
        text(-0.05,-0.6, 'A', 'FontSize',14)
        rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',1)
        text( 2.65,-0.6, 'B', 'FontSize',14)
        rectangle('Position',[1.25 -0.2 .35 .4], 'EdgeColor', 'black', 'LineWidth',1)
        text( 1.25,-0.05, 'Obs', 'FontSize',10)

        subplot(2,2,3)
        plot(obs.pose123.position(:,1), obs.pose123.linvel(:,1), 'LineWidth',1.5);
        hold on;
        plot(obs.pose123.position(:,1), obs.pose123.linvel(:,2), 'LineWidth',1.5);
        plot(obs.pose123.position(:,1), vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);
        hold off;
        grid on; box on;
        xlabel('x-axis, m');
        ylabel('v_y, m/s');
        ylim([-1.4 1.2])
        subtitle('Velocity (y-axis) vs horizontal Distance')
        xts = xticks;
        xticks(xts(1):0.5:xts(end));
        xlim(xl1);


        subplot(2,2,2)
        plot(obs.pose123.time_steps, obs.pose123.linvel(:,1), 'LineWidth',1.5); hold on;
        plot(obs.pose123.time_steps, obs.pose123.linvel(:,2), 'LineWidth',1.5);
        plot(obs.pose123.time_steps, vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);hold off;
        hL = legend({'x', 'y', '||xy||'},'Location','southwest','NumColumns',4);
        grid on; box on;
        xlabel('time, sec');
        ylabel('v_x, m/s');
        subtitle('Velocity in x-axis vs time')
        xts = xticks;
        xticks(xts(1):xts(end));


        subplot(2,2,4)
        plot(obs.pose123.position(:,1), rad2deg(obs.pose123.orientation(:,4)), 'LineWidth',1.5);
        grid on; box on;
        xlabel('x-axis, m');
        ylabel('angle, deg');
        subtitle('Orientation angle vs x-axis')
        xlim(xl1);

        xts = xticks;
        xticks(xts(1):0.5:xts(end));

        sgtitle(sprintf('%s    %s    %s    Kinematics', obs.obs_id, obs.traj_type, obs.motion_type ), ...
                'Interpreter','none');

        set(hL, 'Position',[0.51 0.03 0.01 0.01],'Units','normalized')    
    
    
    
    case 3
        % Mix plots of 2D map, velocity and Fstretch vs space
        
        % (2,2,4) subplot is Fstreatch vs x-axis
        subplot(2,2,1)
        plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth',1.5);
        grid on; axis equal; box on;
        xlabel('x-axis, m');
        ylabel('y-axis, m');
        ylim([-1.4 1.2]);
        subtitle('2D Trajectory')
        xts = xticks;
        xl1 = xlim;
        xticks(xts(1):0.5:xts(end));
        % Plot tables and obstacle
        rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',1)
        text(-0.05,-0.6, 'A', 'FontSize',14)
        rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',1)
        text( 2.65,-0.6, 'B', 'FontSize',14)
        rectangle('Position',[1.25 -0.2 .35 .4], 'EdgeColor', 'black', 'LineWidth',1)
        text( 1.25,-0.05, 'Obs', 'FontSize',10)

        subplot(2,2,3)
        plot(obs.pose123.position(:,1), obs.pose123.linvel(:,1), 'LineWidth',1.5);
        hold on;
        plot(obs.pose123.position(:,1), obs.pose123.linvel(:,2), 'LineWidth',1.5);
        plot(obs.pose123.position(:,1), vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);
        hold off;
        grid on; box on;
        xlabel('x-axis, m');
        ylabel('v_y, m/s');
        ylim([-1.4 1.2])
        subtitle('Velocity (y-axis) vs horizontal Distance')
        xts = xticks;
        xticks(xts(1):0.5:xts(end));
        xlim(xl1);


        subplot(2,2,2)
        plot(obs.pose123.time_steps, obs.pose123.linvel(:,1), 'LineWidth',1.5); hold on;
        plot(obs.pose123.time_steps, obs.pose123.linvel(:,2), 'LineWidth',1.5);
        plot(obs.pose123.time_steps, vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1.5);hold off;
        hL = legend({'x', 'y', '||xy||'},'Location','southwest','NumColumns',4);
        grid on; box on;
        xlabel('time, sec');
        ylabel('v_x, m/s');
        subtitle('Velocity in x-axis vs time')
        xts = xticks;
        xticks(xts(1):xts(end));


        subplot(2,2,4)
%         fxy = vecnorm(obs.fstretch.forceS(:,1:2),2,2);
        fxy = obs.fstretch.force(:,1:2); % use only Fstretch along X-dir
        fxy = fxy(1:10:end,:);
        plot(obs.pose123.position(:,1), fxy, 'LineWidth',1.5);

        grid on; box on;
        xlabel('x-axis, m');
        ylabel('force, N');
        subtitle('Fstretch_{xy} vs x-axis')
        xlim(xl1);

        xts = xticks;
        xticks(xts(1):0.5:xts(end));

        sgtitle(sprintf('%s    %s    %s    Kinematics', obs.obs_id, obs.traj_type, obs.motion_type ), ...
                'Interpreter','none');

        set(hL, 'Position',[0.51 0.03 0.01 0.01],'Units','normalized')
    
    
    
    
    case 4
        
        subplot(2,2,1)
        plot(obs.pose123.position(:,1), obs.pose123.position(:,2), 'LineWidth',1.5);
        grid on; axis equal; box on;
        xlabel('x-axis, m');
        ylabel('y-axis, m');
        ylim([-1.4 1.2]);
        subtitle('2D Trajectory')
        xts = xticks;
        xl1 = xlim;
        xticks(xts(1):0.5:xts(end));
        rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',1)
        text(-0.05,-0.6, 'A', 'FontSize',14)
        rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',1)
        text( 2.65,-0.6, 'B', 'FontSize',14)
        rectangle('Position',[1.25 -0.2 .4 .4], 'EdgeColor', 'black', 'LineWidth',3)
        text( 1.35,-0.05, 'Obs', 'FontSize',12)



        subplot(2,2,3)
        plot(obs.pose123.position(:,1), obs.pose123.linvel(:,1), 'LineWidth',1.5);
        hold on;
        plot(obs.pose123.position(:,1), obs.pose123.linvel(:,2), 'LineWidth',1.5);
        plot(obs.pose123.position(:,1), vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);
        hold off;
        grid on; box on;
        xlabel('x-axis, m');
        ylabel('v_y, m/s');
        ylim([-1.4 1.2])
        subtitle('Velocity (y-axis) vs horizontal Distance')
        xts = xticks;
        xticks(xts(1):0.5:xts(end));
        xlim(xl1);


        subplot(2,2,2)
        plot(obs.pose123.time_steps, obs.pose123.linvel(:,1), 'LineWidth',1.5); hold on;
        plot(obs.pose123.time_steps, obs.pose123.linvel(:,2), 'LineWidth',1.5);
        plot(obs.pose123.time_steps, vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);hold off;
        hL = legend({'x', 'y', '||xy||'},'Location','southwest','NumColumns',4);
        grid on; box on;
        xlabel('time, sec');
        ylabel('v_x, m/s');
        subtitle('Velocity in x-axis vs time')
        xts = xticks;
        xticks(xts(1):xts(end));


        subplot(2,2,4)
        plot(obs.pose123.position(:,2), obs.pose123.linvel(:,1), 'LineWidth',1.5);
        hold on;
        plot(obs.pose123.position(:,2), obs.pose123.linvel(:,2), 'LineWidth',1.5);
        plot(obs.pose123.position(:,2), vecnorm(obs.pose123.linvel(:,1:2),2,2),'k--', 'LineWidth',1);
        hold off
        grid on; box on;
        xlabel('time, sec');
        ylabel('v_y, m/s');
        ylim([-1 1])
        subtitle('Velocity in y-axis vs time')
        xlim([-1.5 1.5])

        xts = xticks;
        xticks(xts(1):0.5:xts(end));

        sgtitle(sprintf('%s    %s    %s    Kinematics', obs.obs_id, obs.traj_type, obs.motion_type ), ...
                'Interpreter','none');

        set(hL, 'Position',[0.51 0.03 0.01 0.01],'Units','normalized')

end

end