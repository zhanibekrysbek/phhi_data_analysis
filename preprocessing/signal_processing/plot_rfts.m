function [] = plot_rfts(obs)

    subplot(221);
    plot(obs.rft1.time_steps,obs.rft1.force)
    subtitle(sprintf('%s Force', obs.rft1.frame_id))
    grid on;
	y1 = ylim;
    ylabel('N')
    
    subplot(223);
    plot(obs.rft1.time_steps,obs.rft1.torque)
    subtitle(sprintf('%s Torque', obs.rft1.frame_id))
    grid on;
    y2 = ylim;
%     legend('x','y','z')
    xlabel('time, s')
    ylabel('Nm')
    
    
    subplot(222);
    plot(obs.rft2.time_steps,obs.rft2.force)
    subtitle(sprintf('%s Force', obs.rft2.frame_id))
    grid on;
    y1 = [y1 ylim];
    legend('x','y','z')

    
    subplot(224);
    plot(obs.rft2.time_steps,obs.rft2.torque)
    subtitle(sprintf('%s Torque', obs.rft2.frame_id))
    grid on;
    y2 = [y2 ylim];
    xlabel('time, s')
    legend('x','y','z')
    
    sgtitle(sprintf('%s %s %s RFT', obs.obs_id, obs.traj_type, obs.motion_type ), ...
        'Interpreter','none');
    
    
    subplot(221);
    ylim([min(y1) max(y1)])
    subplot(222);
    ylim([min(y1) max(y1)])
    
    subplot(223);
    ylim([min(y2), max(y2)])
    subplot(224);
    ylim([min(y2), max(y2)])
    
end