function [] = plot_phase(obs,opt)



switch opt
    case 1
        subplot(2,1,1)
        % Phase Angles
        fsum_ph = atan2d(obs.rft1.force(:,2),obs.rft1.force(:,1));
        % TODO: Acc is in Spatial Frame. Convert to Body
        linacc_ph = atan2d(obs.pose123.linacc(:,2), obs.pose123.linacc(:,2));
        imuacc_ph = atan2d(obs.imu.accel(:,2), obs.imu.accel(:,1));
        
        fs = plot(obs.rft1.time_steps, fsum_ph); hold on;
        lacc = plot(obs.pose123.time_steps, linacc_ph);
        iacc = plot(obs.pose123.time_steps, imuacc_ph);
        
        hold off; grid on;
        ylabel('vector angles, deg');
        
        subplot(2,1,2)
        % Signal Magnitude
        fsm = plot(obs.rft1.time_steps, vecnorm(obs.fsum.force(:,1:2),2,2)); hold on;
        laccm = plot(obs.pose123.time_steps, vecnorm(obs.pose123.linacc(:,1:2),2,2));
        iaccm = plot(obs.pose123.time_steps, vecnorm(obs.imu.accel(:,1:2),2,2));
        
        grid on; hold off;
        
       
end


xlabel('time, sec');

% subtitle('2D Trajectory')
xts = xticks;
% xl1 = xlim;
xticks(xts(1):xts(end));

legend([fs,lacc,iacc], {'fsum', 'linacc', 'imuaccel'});

% legend;
sgtitle(sprintf('%s    %s    %s    Kinematics tlag:%.2d', obs.obs_id, obs.traj_type, obs.motion_type), ...
    'Interpreter','none');





end