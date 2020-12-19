function [] = plot_phase(obs,opt)



switch opt
    case 1
        subplot(2,1,1)
        % Phase Angles
        fsum_ph = atan2d(obs.fsum.force(:,2),obs.fsum.force(:,1));
        fsum_ph = unwrap(fsum_ph);
        % TODO: Acc is in Spatial Frame. Convert to Body
        linacc_ph = atan2d(obs.pose123.linaccB(:,2), obs.pose123.linaccB(:,1));
        linacc_ph = unwrap(linacc_ph);
        imuacc_ph = atan2d(obs.imu.accel(:,2), obs.imu.accel(:,1));
        imuacc_ph = unwrap(imuacc_ph);
        
        
        
        fs = plot(obs.rft1.time_steps, fsum_ph); hold on;
        lacc = plot(obs.pose123.time_steps, linacc_ph);
        iacc = plot(obs.imu.time_steps, imuacc_ph);
        
        hold off; grid on;
        ylabel('vector angles, deg');
        xts = xticks;
        xticks(xts(1):xts(end));
        
        subplot(2,1,2)
        % Signal Magnitude
        fsm = plot(obs.fsum.time_steps, vecnorm(obs.fsum.force(:,1:2),2,2)); hold on;
        laccm = plot(obs.pose123.time_steps, vecnorm(obs.pose123.linaccB(:,1:2),2,2));
        iaccm = plot(obs.imu.time_steps, vecnorm(obs.imu.accel(:,1:2),2,2));
        
        grid on; hold off;
        xts = xticks;
        xticks(xts(1):xts(end));
        ylabel('Magnitude')
end


xlabel('time, sec');



legend([fsm,laccm,iaccm], {'fsum', 'linacc', 'imuaccel'});

% legend;
sgtitle(sprintf('%s    %s    %s  Body Frame', obs.obs_id, obs.traj_type, obs.motion_type), ...
    'Interpreter','none');


end