function [] = plot_imu(obs,opt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


switch opt
    
    case 1
        subplot(3,1,1)
        plot(obs.imu.time_steps, obs.imu.accel);
        subtitle('accel');
        ylabel('m/s^2');
        grid on;
        
        subplot(3,1,2)
        plot(obs.imu.time_steps, obs.imu.gyro);
        subtitle('gyro');
        ylabel('deg/sec');
        grid on;
        
        subplot(3,1,3)
        plot(obs.imu.time_steps, obs.imu.mag);
        subtitle('mag')
        xlabel('time, sec')
        ylabel('\mu T');
        grid on;
        hL = legend({'x', 'y', 'z'},'Location','southwest','NumColumns',3);

        set(hL, 'Position',[0.2 0.03 0.01 0.01],'Units','normalized')
        
        sgtitle(sprintf('%s %s %s IMU', obs.obs_id, obs.traj_type, obs.motion_type ), ...
    'Interpreter','none');
        
        
    case 2
        % plot pose tracking results

        % Position
%         set(gcf,'Position',[1024 768 1024 768])
        
        subplot(2,2,1)
        plot(obs.pose123.time_steps, obs.pose123.position)
        subtitle(sprintf('%s Position', 'Aruco'))
        grid on;
        ylabel('m')

        subplot(2,2,2)
        plot(obs.pose123.time_steps, obs.pose123.orientation);
        subtitle(sprintf('%s Orientation', 'Aruco'))
        grid on
        yt1 = yticks;
        ylabel('axis-angle')

        subplot(2,2,3)
        plot(obs.imu.time_steps, obs.imu.angvel)
        subtitle(sprintf('%s Angular Velocty', 'KF'))
        grid on
        xlabel('time, sec');
        ylabel('rad/sec')

        subplot(2,2,4)
        plot(obs.imu.time_steps, obs.imu.orientation)
        subtitle(sprintf('%s Orientation', 'KF'))
        yt2 = yticks;
        grid on
        xlabel('time, sec');
        ylabel('axis-angle')
        
        hL = legend({'x', 'y', 'z','angle'},'Location','southwest','NumColumns',4);
        
        set(hL, 'Position',[0.51 0.03 0.01 0.01],'Units','normalized')
        
        sgtitle(sprintf('%s %s %s Orient Tracking', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');
        
        yts = min(yt1(1), yt2(1)):0.5:max(yt1(end), yt2(end));
        
        
        subplot(2,2,2);
        yticks(yts);
        subplot(2,2,4);
        yticks(yts);
        
end



end

