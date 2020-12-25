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
        
end

sgtitle(sprintf('%s %s %s RFT', obs.obs_id, obs.traj_type, obs.motion_type ), ...
    'Interpreter','none');

end

