function [] = plot_phase(obs,opt)



switch opt
    case 1
        % Phase Angles in Body Frame
        subplot(2,1,1)
        % Phase Angles
        fsum_ph = atan2d(obs.fsum.force(:,2),obs.fsum.force(:,1));
        linacc_ph = atan2d(obs.pose123.linaccB(:,2), obs.pose123.linaccB(:,1));
        imuacc_ph = atan2d(obs.imu.accel(:,2), obs.imu.accel(:,1));
        
        fs = plot(obs.rft1.time_steps, fsum_ph,'.'); hold on;
        lacc = plot(obs.pose123.time_steps, linacc_ph, '.');
        iacc = plot(obs.imu.time_steps, imuacc_ph, '.');
        
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
        
        xlabel('time, sec');
        legend([fsm,laccm,iaccm], {'fsum', 'linacc', 'imuaccel'});
        % legend;
        sgtitle(sprintf('%s    %s    %s  Body Frame', obs.obs_id, obs.traj_type, obs.motion_type), ...
            'Interpreter','none');
        
    case 2
        
        % Phase Angles in Spatial Frame
        subplot(2,1,1)
        fsum_ph = atan2d(obs.fsum.forceS(:,2),obs.fsum.forceS(:,1));
        
        linacc_ph = atan2d(obs.pose123.linacc(:,2), obs.pose123.linacc(:,1));
        
        imuacc_ph = atan2d(obs.imu.accelS(:,2), obs.imu.accelS(:,1));
        
        
        fs = plot(obs.rft1.time_steps, fsum_ph,'.'); hold on;
        lacc = plot(obs.pose123.time_steps, linacc_ph,'.');
        iacc = plot(obs.imu.time_steps, imuacc_ph,'.');
        
        hold off; grid on;
        ylabel('vector angles, deg');
        xts = xticks;
        xticks(xts(1):xts(end));
        
        subplot(2,1,2)
        % Signal Magnitude
        fsm = plot(obs.fsum.time_steps, vecnorm(obs.fsum.forceS(:,1:2),2,2)); hold on;
        laccm = plot(obs.pose123.time_steps, vecnorm(obs.pose123.linacc(:,1:2),2,2));
        iaccm = plot(obs.imu.time_steps, vecnorm(obs.imu.accelS(:,1:2),2,2));
        
        grid on; hold off;
        xts = xticks;
        xticks(xts(1):xts(end));
        ylabel('Magnitude in xy-plane')
        
        
        xlabel('time, sec');
        legend([fsm,laccm,iaccm], {'fsum', 'linacc', 'imuaccel'});
        % legend;
        sgtitle(sprintf('%s    %s    %s  Spatial Frame', obs.obs_id, obs.traj_type, obs.motion_type), ...
            'Interpreter','none');
    
    case 3
        % All dofs
        subplot(5,1,1);
        plot(obs.rft1.time_steps, obs.fsum.force(:,1)); hold on;
        plot(obs.imu.time_steps, obs.imu.accel(:,1)); hold off;
        subtitle('x - axis')
        ylabel('N | m/s^2');
        grid on;

        subplot(5,1,2);
        plot(obs.rft1.time_steps, obs.fsum.force(:,2)); hold on;
        plot(obs.imu.time_steps, obs.imu.accel(:,2)); hold off;
        subtitle('y - axis')
        ylabel('N | m/s^2');
        grid on;

        subplot(5,1,3);
        f1 = plot(obs.rft1.time_steps, obs.fsum.force(:,3)); hold on;
        a1 = plot(obs.imu.time_steps, obs.imu.accel(:,3)); hold off;
        subtitle('z - axis');
        ylabel('N | m/s^2');
        grid on;
        
        subplot(5,1,4);
        % Signal Magnitude
        plot(obs.fsum.time_steps, vecnorm(obs.fsum.force(:,1:2),2,2)); hold on;
        plot(obs.imu.time_steps, vecnorm(obs.imu.accel(:,1:2),2,2)); hold off;
        grid on;
        subtitle('magnitude')

        subplot(5,1,5);
        % Phase Angles
        fsum_ph = atan2d(obs.fsum.force(:,2),obs.fsum.force(:,1));
        imuacc_ph = atan2d(obs.imu.accel(:,2), obs.imu.accel(:,1));

        fs = plot(obs.rft1.time_steps, fsum_ph,'.','MarkerSize',1); hold on;
        iacc = plot(obs.imu.time_steps, imuacc_ph,'.','MarkerSize',1); hold off;
        subtitle('Phase in xy-plane')

        hold off; grid on;
        ylabel('vector angles, deg');
        xts = xticks;
        xticks(xts(1):xts(end));
        
        hL = legend([f1,a1], {'force', 'accel'}, 'NumColumns', 2);
        sgtitle(sprintf('Accel vs Fsum. %s    %s    %s  Body Frame', obs.obs_id, obs.traj_type, obs.motion_type), ...
            'Interpreter','none');
        set(hL, 'Position',[0.51 0.03 0.01 0.01],'Units','normalized')

end





end


% function 