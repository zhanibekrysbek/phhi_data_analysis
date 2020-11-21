function [] = plot_rfts(obs,opt)
    if opt==1
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
        
    elseif opt == 2
    
        subplot(321);
        plot(obs.rft1.time_steps, obs.rft1.force); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.rft1.force(:,1:2),2,2),'k--'); hold off;

        grid on;
        subtitle('Force 1 ');

        subplot(322);
        plot(obs.rft1.time_steps, torque_s);
        grid on;
        subtitle('Torque 1 ');

        subplot(323);
        plot(obs.rft1.time_steps, force_s_1);
        grid on;
        subtitle('Force 2 ');


        subplot(324);
        plot(obs.rft1.time_steps, torque_s_1);
        grid on;
        subtitle('Torque 2');
        legend('x','y','z');

        subplot(325);
        plot(obs.rft1.time_steps, force_sum(:,1:2), obs.rft1.time_steps, vecnorm(force_sum(:,1:2),2,2));
        grid on;
        subtitle('Force Sum');


        subplot(326);
        plot(obs.rft1.time_steps, force_s(:,1:2)-force_s_1(:,1:2)); hold on;
        plot(obs.rft1.time_steps, vecnorm(force_s(:,1:2)-force_s_1(:,1:2),2,2), 'k--'); hold off
        grid on;
        subtitle('F stretch');
        legend('x','y','xy');
    end
    
end